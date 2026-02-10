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
        self.frame_queue = Queue(maxsize=5)  # Keep small to avoid latency
        self.detector = DetectionEngine(model_path='/home/pi/Adeept_DarkPaw/models/yolov11m.hef',
                                        score_thresh=0.7,
                                        max_detections=3)
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

        self.detection_thread = Thread(target=self.detector.run_forever)
        self.detection_thread.start()

        self.dmn_thread = Thread(target=self.dmn.run)
        self.dmn_thread.start()

        # Initialize GStreamer
        Gst.init(None)

        # Web app setup
        self.app = web.Application()
        self.app.router.add_get('/', self.index)
        self.app.router.add_get('/static/scripts.js', self.javascript)
        self.app.router.add_get('/static/jquery-3.2.1.min.js', self.jquery)
        self.app.router.add_get('/static/style.css', self.styles)
        self.app.router.add_get('/ws', self.websocket_handler)

    # Simulated data source
    def fake_data_updater(self):
        import random
        while self.detector.running:
            time.sleep(0.2)
            data = {
                "Distance": f"{random.randint(20, 100)}",
                "CPU_temp": f"{random.uniform(40.0, 60.0):.1f}",
                "CPU_load": f"{random.uniform(10.0, 90.0):.1f}",
                "RAM_usage": f"{random.uniform(20.0, 80.0):.1f}"
            }
            self.data_queue.put(data)

    # === Web Routes ===
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

    # === WebSocket/WebRTC ===
    async def websocket_handler(self, request):
        loop = asyncio.get_running_loop()
        ws = web.WebSocketResponse()
        await ws.prepare(request)

        # --- GStreamer pipeline ---
        pipeline = Gst.Pipeline.new("webrtc-pipeline")
        src = Gst.ElementFactory.make("appsrc", "source")
        conv = Gst.ElementFactory.make("videoconvert", "convert")
        encoder = Gst.ElementFactory.make("vp8enc", "encoder")
        payloader = Gst.ElementFactory.make("rtpvp8pay", "pay")
        webrtc = Gst.ElementFactory.make("webrtcbin", "sendrecv")

        for elem in [src, conv, encoder, payloader, webrtc]:
            pipeline.add(elem)

        src.link(conv)
        conv.link(encoder)
        encoder.link(payloader)

        # --- appsrc properties ---
        src.set_property("is-live", True)
        src.set_property("format", Gst.Format.TIME)
        src.set_property("block", True)
        src.set_property("caps", Gst.Caps.from_string(
            "video/x-raw,format=BGRx,width=800,height=600,framerate=30/1"
        ))

        encoder.set_property("deadline", 1)
        encoder.set_property("end-usage", 1)
        encoder.set_property("target-bitrate", 1000000)
        webrtc.set_property("bundle-policy", "max-bundle")

        # --- Dynamic linking from payloader to webrtcbin ---
        def on_pad_added(payloader, pad):
            sink_pad = webrtc.get_request_pad("sink_%u")
            if sink_pad:
                pad.link(sink_pad)
                logger.info("✅ Linked payloader to webrtcbin")

        payloader.connect("pad-added", on_pad_added)

        # --- Add video transceiver ---
        caps = Gst.Caps.from_string("application/x-rtp,media=video,encoding-name=VP8,payload=96")
        webrtc.emit("add-transceiver", GstWebRTC.WebRTCRTPTransceiverDirection.SENDONLY, caps)

        # --- Frame pusher thread ---
        def feed_frame(request):
            frame = request.make_array("main")
            if self.frame_queue.full():
                try:
                    self.frame_queue.get_nowait()
                except Empty:
                    pass
            self.frame_queue.put_nowait(frame)

        def frame_pusher():
            while True:
                if not self.pipeline_ready:
                    time.sleep(0.01)
                    continue

                try:
                    frame = self.frame_queue.get(timeout=1)
                except Empty:
                    continue

                if frame is None:
                    continue

                # Postprocessing
                frame_with_detections = self.detector.postprocess_frames(frame)
                frame_data = frame_with_detections.tobytes()

                buf = Gst.Buffer.new_allocate(None, len(frame_data), None)
                buf.fill(0, frame_data)
                buf.pts = buf.dts = int(self.frame_count * Gst.SECOND / 30)
                buf.duration = int(Gst.SECOND / 30)
                self.frame_count += 1

                ret = src.emit("push-buffer", buf)
                if ret != Gst.FlowReturn.OK:
                    logger.warning(f"❌ Failed to push buffer: {ret}")

        Thread(target=frame_pusher, daemon=True).start()

        # --- Start camera ---
        if not self.camera_lock.acquire(blocking=False):
            logger.info("❌ Camera already in use")
            return web.Response(text="Camera busy", status=503)

        self.picam2.pre_callback = feed_frame
        self.picam2.start()

        # --- WebRTC negotiation ---
        self.pcs.add(ws)

        def setup_data_channel():
            if self.data_channel_set_up:
                return
            data_channel = webrtc.emit("create-data-channel", "control", None)
            if not data_channel:
                logger.warning("❌ Could not create data channel")
                return

            logger.info("📡 Server created data channel")

            def on_open(channel):
                logger.info("✅ Server data channel is open")

            def on_message(channel, message):
                if message == "request_status":
                    if not self.data_queue.empty():
                        data = self.data_queue.get()
                        data["type"] = "status_update"
                        channel.emit("send-string", json.dumps(data))
                else:
                    if self.command_queue.full():
                        try:
                            self.command_queue.get_nowait()
                        except Empty:
                            pass
                    self.command_queue.put_nowait(message)
                    logger.info(f"✅ Command queued: {message}")

            data_channel.connect("on-open", on_open)
            data_channel.connect("on-message-string", on_message)
            self.data_channel_set_up = True

        def on_negotiation_needed(element):
            if self.negotiation_in_progress:
                return

            self.negotiation_in_progress = True

            def on_offer_created(promise, ws_conn, _user_data):
                reply = promise.get_reply()
                offer = reply.get_value("offer")
                webrtc.emit("set-local-description", offer, None)

                # Send SDP to client
                sdp_msg = json.dumps({'sdp': {'type': 'offer', 'sdp': offer.sdp.as_text()}})
                asyncio.run_coroutine_threadsafe(ws.send_str(sdp_msg), loop)

                # ✅ Ready to push frames
                self.pipeline_ready = True
                logger.info("✅ Pipeline ready, frame pusher active")
                setup_data_channel()
                self.negotiation_in_progress = False

            promise = Gst.Promise.new_with_change_func(on_offer_created, ws, None)
            webrtc.emit("create-offer", None, promise)

        def on_ice_candidate(_, mlineindex, candidate):
            ice_msg = json.dumps({'ice': {'candidate': candidate, 'sdpMLineIndex': mlineindex}})
            asyncio.run_coroutine_threadsafe(ws.send_str(ice_msg), loop)

        webrtc.connect("on-negotiation-needed", on_negotiation_needed)
        webrtc.connect("on-ice-candidate", on_ice_candidate)

        pipeline.set_state(Gst.State.PLAYING)

        # --- WebSocket message loop ---
        try:
            async for msg in ws:
                if msg.type != web.WSMsgType.TEXT:
                    continue

                data = json.loads(msg.data)
                if 'sdp' in data:
                    sdp = data['sdp']
                    res, sdpmsg = GstSdp.SDPMessage.new_from_text(sdp['sdp'])
                    if res != GstSdp.SDPResult.OK:
                        logger.warning("❌ Failed to parse SDP answer")
                        continue
                    answer = GstWebRTC.WebRTCSessionDescription.new(
                        GstWebRTC.WebRTCSDPType.ANSWER, sdpmsg)
                    webrtc.emit('set-remote-description', answer, None)
                    setup_data_channel()
                elif 'ice' in data:
                    ice = data['ice']
                    webrtc.emit('add-ice-candidate', ice['sdpMLineIndex'], ice['candidate'])

        except Exception as e:
            logger.warning(f"WebSocket error: {e}")

        finally:
            # Cleanup
            logger.info("🛑 Cleaning up...")
            try:
                if pipeline:
                    pipeline.set_state(Gst.State.NULL)
                    pipeline.get_state(Gst.CLOCK_TIME_NONE)
                    pipeline = None
            except Exception as e:
                logger.warning(f"❌ Failed to stop pipeline: {e}")
            self.stop_camera()
            self.data_channel_set_up = False
            self.pipeline_ready = False
            self.frame_count = 0

        return ws

    # === Cleanup / stop camera ===
    def stop_camera(self):
        try:
            self.picam2.stop()
            logger.info("📷 Picamera2 stopped.")
        except Exception as e:
            logger.error(f"Failed to stop Picamera2: {e}")
        if self.camera_lock.locked():
            self.camera_lock.release()

    async def cleanup(self, app):
        logger.info("🛑 Graceful shutdown...")
        self.detector.stop()
        self.detection_thread.join(timeout=2)
        self.dmn.shutdown()
        self.dmn_thread.join(timeout=2)
        self.stop_camera()
        for ws in self.pcs:
            await ws.close()
        logger.info("✅ Shutdown complete.")

    async def start_background(self, host='0.0.0.0', port=4664):
        self.runner = web.AppRunner(self.app)
        await self.runner.setup()
        self.site = web.TCPSite(self.runner, host, port)
        await self.site.start()
        logger.info(f"🚀 WebServer started at http://{host}:{port}")

    async def stop_background(self):
        await self.cleanup(self.app)
        await self.runner.cleanup()


if __name__ == '__main__':
    try:
        webserver = WebServer()
        webserver.app.on_shutdown.append(webserver.cleanup)
        web.run_app(webserver.app, port=4664)
    except KeyboardInterrupt:
        logger.info("🛑 KeyboardInterrupt received. Exiting...")
