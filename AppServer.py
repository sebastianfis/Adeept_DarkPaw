import asyncio
import json
from aiohttp import web
import logging
from threading import Thread, Lock
import gi
import time
from queue import Queue, Empty
gi.require_version('Gst', '1.0')
gi.require_version('GstWebRTC', '1.0')
from gi.repository import Gst, GstWebRTC, GObject, GstSdp
from detection_engine import DetectionEngine
from DMN import DefaultModeNetwork

logging.basicConfig(level=logging.INFO)
logger = logging.getLogger(__name__)

class WebServer:
    def __init__(self):
        self.data_queue = Queue(maxsize=2)
        self.command_queue = Queue(maxsize=1)
        self.frame_queue = Queue(maxsize=5)
        self.detector = DetectionEngine(
            model_path='/home/pi/Adeept_DarkPaw/models/yolov11m.hef',
            score_thresh=0.7,
            max_detections=3
        )
        self.dmn = DefaultModeNetwork(self.detector, self.data_queue, self.command_queue)

        self.pcs = set()
        self.picam2 = self.detector.camera
        self.camera_lock = Lock()
        self.frame_count = 0
        self.data_channel_set_up = False
        self.negotiation_in_progress = False
        self.pipeline_ready = False
        self.runner = None
        self.site = None

        # Start detection and DMN threads
        self.detection_thread = Thread(target=self.detector.run_forever, daemon=True)
        self.detection_thread.start()
        self.dmn_thread = Thread(target=self.dmn.run, daemon=True)
        self.dmn_thread.start()

        Gst.init(None)

        # Web app
        self.app = web.Application()
        self.app.router.add_get('/', self.index)
        self.app.router.add_get('/static/scripts.js', self.javascript)
        self.app.router.add_get('/static/jquery-3.2.1.min.js', self.jquery)
        self.app.router.add_get('/static/style.css', self.styles)
        self.app.router.add_get('/ws', self.websocket_handler)

    # Web routes
    @staticmethod
    async def index(request):
        return web.FileResponse('templates/index.html')

    @staticmethod
    async def javascript(request):
        return web.FileResponse('static/scripts.js')

    @staticmethod
    async def jquery(request):
        return web.FileResponse('static/jquery-3.2.1.min.js')

    @staticmethod
    async def styles(request):
        return web.FileResponse('static/style.css')

    async def websocket_handler(self, request):
        loop = asyncio.get_running_loop()
        ws = web.WebSocketResponse()
        await ws.prepare(request)

        # Build pipeline
        pipeline = Gst.Pipeline.new("webrtc-pipeline")
        src = Gst.ElementFactory.make("appsrc", "source")
        conv = Gst.ElementFactory.make("videoconvert", "convert")
        scale = Gst.ElementFactory.make("videoscale", "scale")
        caps = Gst.ElementFactory.make("capsfilter", "caps")
        encoder = Gst.ElementFactory.make("vp8enc", "encoder")
        payloader = Gst.ElementFactory.make("rtpvp8pay", "pay")
        webrtc = Gst.ElementFactory.make("webrtcbin", "sendrecv")

        if not all([src, conv, scale, caps, encoder, payloader, webrtc]):
            raise RuntimeError("Missing GStreamer elements. Check plugins!")

        # Configure appsrc
        src.set_property("is-live", True)
        src.set_property("format", Gst.Format.TIME)
        src.set_property("block", True)
        src.set_property("caps", Gst.Caps.from_string(
            "video/x-raw,format=BGRx,width=800,height=600,framerate=30/1"
        ))

        encoder.set_property("deadline", 1)
        encoder.set_property("end-usage", 1)
        encoder.set_property("target-bitrate", 1000000)

        for elem in [src, conv, scale, caps, encoder, payloader, webrtc]:
            pipeline.add(elem)

        src.link(conv)
        conv.link(scale)
        scale.link(caps)
        caps.link(encoder)
        encoder.link(payloader)

        # === Start camera ===
        if not self.camera_lock.acquire(blocking=False):
            logger.info("❌ Camera already in use")
            return web.Response(text="Camera busy", status=503)

        def feed_frame(request):
            frame = request.make_array("main")
            if self.frame_queue.full():
                try: self.frame_queue.get_nowait()
                except Empty: pass
            self.frame_queue.put_nowait(frame)

        self.picam2.pre_callback = feed_frame
        self.picam2.start()

        # === Frame pusher thread ===
        def frame_pusher():
            while True:
                frame = self.frame_queue.get()
                if not self.pipeline_ready:
                    continue  # wait until payloader → webrtcbin linked
                frame_proc = self.detector.postprocess_frames(frame)
                data = frame_proc.tobytes()
                buf = Gst.Buffer.new_allocate(None, len(data), None)
                buf.fill(0, data)
                buf.pts = buf.dts = int(self.frame_count * Gst.SECOND / 30)
                buf.duration = int(Gst.SECOND / 30)
                self.frame_count += 1
                ret = src.emit("push-buffer", buf)
                if ret != Gst.FlowReturn.OK:
                    logger.warning(f"Failed to push buffer: {ret}")

        Thread(target=frame_pusher, daemon=True).start()

        # === WebRTC setup ===
        self.pcs.add(ws)

        def setup_data_channel():
            if self.data_channel_set_up: return
            channel = webrtc.emit("create-data-channel", "control", None)
            if not channel:
                logger.error("❌ Could not create data channel")
                return

            def on_open(ch): logger.info("✅ Data channel open")
            def on_message(ch, msg):
                if msg == "request_status" and not self.data_queue.empty():
                    data = self.data_queue.get()
                    data["type"] = "status_update"
                    ch.emit("send-string", json.dumps(data))
                else:
                    if self.command_queue.full():
                        try: self.command_queue.get_nowait()
                        except Empty: pass
                    self.command_queue.put_nowait(msg)
                    logger.info(f"✅ Command queued: {msg}")

            channel.connect("on-open", on_open)
            channel.connect("on-message-string", on_message)
            self.data_channel_set_up = True

        def on_negotiation_needed(element):
            if self.negotiation_in_progress: return
            self.negotiation_in_progress = True

            payloader_src = payloader.get_static_pad("src")
            webrtc_sink = webrtc.get_request_pad("sink_%u")
            if payloader_src.link(webrtc_sink) == Gst.PadLinkReturn.OK:
                logger.info("✅ Linked payloader to webrtc")
                self.pipeline_ready = True  # now safe to push frames
                pipeline.set_state(Gst.State.PLAYING)
            else:
                logger.error("❌ Failed to link payloader to webrtc")

            promise = Gst.Promise.new_with_change_func(on_offer_created, ws, None)
            webrtc.emit("create-offer", None, promise)

        def on_offer_created(promise, ws_conn, _user_data):
            reply = promise.get_reply()
            offer = reply.get_value("offer")
            webrtc.emit("set-local-description", offer, None)
            asyncio.run_coroutine_threadsafe(
                ws.send_str(json.dumps({'sdp': {'type':'offer','sdp':offer.sdp.as_text()}})),
                loop
            )
            self.negotiation_in_progress = False
            setup_data_channel()

        def on_ice_candidate(_, mlineindex, candidate):
            ice_msg = json.dumps({'ice': {'candidate': candidate, 'sdpMLineIndex': mlineindex}})
            asyncio.run_coroutine_threadsafe(ws.send_str(ice_msg), loop)

        webrtc.connect('on-negotiation-needed', on_negotiation_needed)
        webrtc.connect('on-ice-candidate', on_ice_candidate)

        pipeline.set_state(Gst.State.PLAYING)

        try:
            async for msg in ws:
                if msg.type != web.WSMsgType.TEXT: continue
                data = json.loads(msg.data)

                if 'sdp' in data:
                    sdp = data['sdp']
                    res, sdpmsg = GstSdp.SDPMessage.new_from_text(sdp['sdp'])
                    if res != GstSdp.SDPResult.OK: continue
                    answer = GstWebRTC.WebRTCSessionDescription.new(GstWebRTC.WebRTCSDPType.ANSWER, sdpmsg)
                    webrtc.emit("set-remote-description", answer, None)
                    setup_data_channel()

                elif 'ice' in data:
                    ice = data['ice']
                    webrtc.emit("add-ice-candidate", ice['sdpMLineIndex'], ice['candidate'])

        finally:
            logger.info("🛑 Cleaning up...")
            if pipeline:
                pipeline.set_state(Gst.State.NULL)
                pipeline.get_state(Gst.CLOCK_TIME_NONE)
                pipeline = None
            self.stop_camera()
            self.data_channel_set_up = False
            self.pipeline_ready = False
            self.frame_count = 0

        return ws

    async def cleanup(self, app):
        logger.info("🛑 Graceful shutdown...")
        self.detector.stop()
        self.detection_thread.join(timeout=2)
        self.dmn.shutdown()
        self.dmn_thread.join(timeout=2)
        self.stop_camera()
        for ws in self.pcs:
            await ws.close()
        logger.info("✅ Shutdown complete")

    def stop_camera(self):
        try: self.picam2.stop()
        except Exception as e: logger.error(e)
        if self.camera_lock.locked(): self.camera_lock.release()

    async def start_background(self, host='0.0.0.0', port=4664):
        self.runner = web.AppRunner(self.app)
        await self.runner.setup()
        self.site = web.TCPSite(self.runner, host, port)
        await self.site.start()
        logger.info(f"🚀 WebServer running at http://{host}:{port}")

    async def stop_background(self):
        await self.cleanup(self.app)
        await self.runner.cleanup()
        logger.info("✅ WebServer stopped")

if __name__ == "__main__":
    try:
        webserver = WebServer()
        webserver.app.on_shutdown.append(webserver.cleanup)
        web.run_app(webserver.app, port=4664)
    except KeyboardInterrupt:
        logger.info("🛑 KeyboardInterrupt, exiting")
