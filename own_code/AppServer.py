import asyncio
import json
from aiohttp import web
import logging
from threading import Thread, Lock
from multiprocessing import Process
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
        self.frame_queue = Queue(maxsize=5)  # Keep it small to avoid latency
        self.detector = DetectionEngine(model_path='/home/pi/Adeept_DarkPaw/own_code/models/yolov11m.hef',
                                        score_thresh=0.7,
                                        max_detections=3)
        self.dmn = DefaultModeNetwork(self.detector, self.data_queue, self.command_queue)

        self.pcs = set()  # Peer connections
        self.picam2 = self.detector.camera  # Global camera instance
        self.camera_lock = Lock()
        self.frame_count = 0
        self.data_channel_set_up = False
        self.negotiation_in_progress = False
        self.pipeline_ready = False
        self.runner = None
        self.site = None

        self.detection_thread = Process(target=self.detector.run_forever)
        self.detection_thread.start()

        self.dmn_thread = Process(target=self.dmn.run)
        self.dmn_thread.start()

        # Initialize GStreamer
        Gst.init(None)

        # === App setup ===

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
        return web.FileResponse('./templates/index.html')

    @staticmethod
    async def javascript(request):
        return web.FileResponse('./static/scripts.js')

    @staticmethod
    async def jquery(request):
        return web.FileResponse('./static/jquery-3.2.1.min.js')

    @staticmethod
    async def styles(request):
        return web.FileResponse('./static/style.css')

    # === WebSocket/WebRTC handler ===
    async def websocket_handler(self, request):

        loop = asyncio.get_running_loop()
        ws = web.WebSocketResponse()
        await ws.prepare(request)

        pipeline = Gst.Pipeline.new("webrtc-pipeline")

        # GStreamer Elements
        src = Gst.ElementFactory.make("appsrc", "source")
        conv = Gst.ElementFactory.make("videoconvert", "convert")
        scale = Gst.ElementFactory.make("videoscale", "scale")
        caps = Gst.ElementFactory.make("capsfilter", "caps")
        encoder = Gst.ElementFactory.make("vp8enc", "encoder")
        payloader = Gst.ElementFactory.make("rtpvp8pay", "pay")
        webrtc = Gst.ElementFactory.make("webrtcbin", "sendrecv")

        # Setup appsrc caps
        src.set_property("is-live", True)
        src.set_property("format", Gst.Format.TIME)
        src.set_property("block", True)
        src.set_property("caps", Gst.Caps.from_string(
            "video/x-raw,format=BGRx,width=800,height=600,framerate=30/1"))

        # Other element properties

        encoder.set_property("deadline", 1)
        encoder.set_property("end-usage", 1)  # CBR
        encoder.set_property("target-bitrate", 1000000)  # ~1 Mbps

        for elem in [src, conv, scale, caps, encoder, payloader, webrtc]:
            pipeline.add(elem)

        src.link(conv)
        conv.link(scale)
        scale.link(caps)
        caps.link(encoder)
        encoder.link(payloader)
        payloader_src = payloader.get_static_pad("src")
        webrtc_sink = webrtc.get_request_pad("sink_%u")

        if payloader_src.link(webrtc_sink) != Gst.PadLinkReturn.OK:
            logger.info("❌ Failed to link payloader to webrtcbin")
        else:
            logger.info("✅ Linked payloader to webrtcbin")

        # === Picamera2 setup ===
        if not self.camera_lock.acquire(blocking=False):
            logger.info("❌ Camera already in use, rejecting connection.")
            return web.Response(text="Camera busy", status=503)

        def feed_frame(request):
            frame = request.make_array("main")
            if self.frame_queue.full():
                try:
                    self.frame_queue.get_nowait()  # Drop the oldest frame to prevent queue backup
                except Empty:
                    pass
            self.frame_queue.put_nowait(frame)

        def frame_pusher():
            while True:
                try:
                    frame = self.frame_queue.get()  # Wait max 1 sec for a frame
                except Empty:
                    continue  # No frame, just loop

                if pipeline is None:
                    logger.info("⚠ Pipeline is None, skipping frame.")
                    return

                if not self.pipeline_ready:
                    # Check once if pipeline is ready
                    state = pipeline.get_state(0).state
                    if state == Gst.State.PLAYING:
                        logger.info("✅ Pipeline is PLAYING, starting to feed frames.")
                        self.pipeline_ready = True
                    else:
                        logger.info("❌ Pipeline not in PLAYING state. Skipping frame.")
                        return  # Skip pushing frame if pipeline is not ready yet

                # (Postprocess your frame here)
                frame_with_detections = self.detector.postprocess_frames(frame)
                frame_data = frame_with_detections.tobytes()

                buf = Gst.Buffer.new_allocate(None, len(frame_data), None)
                buf.fill(0, frame_data)

                buf.pts = buf.dts = int(self.frame_count * Gst.SECOND / 30)
                buf.duration = int(Gst.SECOND / 30)
                self.frame_count += 1

                ret = src.emit("push-buffer", buf)
                if ret != Gst.FlowReturn.OK:
                    logger.info(f"❌ Failed to push buffer into GStreamer:{ret}")

        pusher_thread = Process(target=frame_pusher, daemon=True)
        pusher_thread.start()

        self.picam2.pre_callback = feed_frame
        self.picam2.start()

        # === WebRTC setup ===
        self.pcs.add(ws)

        def setup_data_channel():
            if self.data_channel_set_up:
                logger.info("❌ Data channel already set up!")
                return  # Prevent re-setup of the data channel

            data_channel = webrtc.emit("create-data-channel", "control", None)
            if not data_channel:
                logger.info("❌ Could not create data channel!")
                return
            logger.info("📡 Server created data channel")

            def on_open(channel):
                logger.info("✅ Server data channel is now open")

            def on_message(channel, message):
                # logger.info(f"📥 Received message on data channel:{message}")

                if message == "request_status":
                    if not self.data_queue.empty():
                        data = self.data_queue.get()
                        data["type"] = "status_update"
                        json_data = json.dumps(data)

                        channel.emit("send-string", json_data)  # <-- send directly!
                    else:
                        logger.info("⚠ Data queue empty, nothing to send.")
                else:
                    if self.command_queue.full():
                        try:
                            self.command_queue.get_nowait()  # Drop the oldest frame to prevent queue backup
                        except Empty:
                            pass
                    self.command_queue.put_nowait(message)
                    logger.info(f"✅ Command queued: {message}")

            data_channel.connect("on-open", on_open)
            data_channel.connect("on-message-string", on_message)
            self.data_channel_set_up = True  # Mark the data channel as set up

        def on_negotiation_needed(element):
            # Check if a negotiation is already in progress
            if self.negotiation_in_progress:
                logger.info("❌ Negotiation already in progress, skipping offer creation.")
                return

            logger.info("Negotiation needed")
            self.negotiation_in_progress = True  # Set the flag to indicate negotiation is in progress

            # Create a promise and handle the offer creation
            promise = Gst.Promise.new_with_change_func(on_offer_created, ws, None)
            webrtc.emit('create-offer', None, promise)

        def on_offer_created(promise, ws_conn, _user_data):
            logger.info("Offer created")
            reply = promise.get_reply()
            offer = reply.get_value("offer")
            webrtc.emit('set-local-description', offer, None)

            # Send the offer SDP to the client
            sdp_msg = json.dumps({'sdp': {
                'type': 'offer',
                'sdp': offer.sdp.as_text()
            }})
            asyncio.run_coroutine_threadsafe(ws.send_str(sdp_msg), loop)

            self.negotiation_in_progress = False  # Reset the flag after the offer is sent

            # Create the data channel after the offer is sent
            setup_data_channel()

        def on_ice_candidate(_, mlineindex, candidate):
            logger.info(f"Python sending ICE: {candidate}")
            ice_msg = json.dumps({'ice': {
                'candidate': candidate,
                'sdpMLineIndex': mlineindex,
            }})
            asyncio.run_coroutine_threadsafe(ws.send_str(ice_msg), loop)

        webrtc.connect('on-negotiation-needed', on_negotiation_needed)
        webrtc.connect('on-ice-candidate', on_ice_candidate)

        pipeline.set_state(Gst.State.PLAYING)

        try:
            async for msg in ws:
                if msg.type == web.WSMsgType.TEXT:
                    data = json.loads(msg.data)

                    if 'sdp' in data:
                        sdp = data['sdp']
                        res, sdpmsg = GstSdp.SDPMessage.new_from_text(sdp['sdp'])
                        if res != GstSdp.SDPResult.OK:
                            logger.info("❌ Failed to parse SDP answer")
                            return
                        answer = GstWebRTC.WebRTCSessionDescription.new(GstWebRTC.WebRTCSDPType.ANSWER, sdpmsg)
                        webrtc.emit('set-remote-description', answer, None)

                        # Only set up the data channel once
                        setup_data_channel()

                    elif 'ice' in data:
                        ice = data['ice']
                        webrtc.emit('add-ice-candidate', ice['sdpMLineIndex'], ice['candidate'])

        except Exception as e:
            logger.info(f"WebSocket error: {e}")

        finally:
            # This runs both after an exception OR after clean disconnection
            logger.info("🛑 Cleaning up...")
            try:
                if pipeline:
                    logger.info("🛑 Stopping pipeline...")
                    pipeline.set_state(Gst.State.NULL)
                    pipeline.get_state(Gst.CLOCK_TIME_NONE)  # Block until NULL
                    pipeline = None  # Fully dereference pipeline
                    logger.info("✅ Pipeline fully stopped and cleaned.")
            except Exception as e:
                logger.info(f"❌ Failed to fully stop pipeline: {e}")
            self.stop_camera()

            # ✅ Reset your flags here
            self.data_channel_set_up = False
            self.pipeline_ready = False
            self.frame_count = 0

            logger.info("🔄 Reset data_channel_set_up flag")

        return ws

    async def cleanup(self, app):
        logger.info("🛑 Graceful shutdown initiated...")

        # Stop threads
        logger.info("🛑 Stopping threads...")
        self.detector.stop()  # Make sure your DetectionEngine has a way to stop cleanly
        self.detection_thread.join(timeout=2)

        self.stop_camera()

        # Close peer connections if any
        for ws in self.pcs:
            await ws.close()

        logger.info("✅ Graceful shutdown complete.")

    def stop_camera(self):
        # Stop Picamera2
        try:
            self.picam2.stop()
            logger.info("📷 Picamera2 stopped.")
        except Exception as e:
            logger.error(f"Failed to stop Picamera2: {e}")

        # Release locks
        if self.camera_lock.locked():
            self.camera_lock.release()

    async def start_background(self, host='0.0.0.0', port=4664):
        print("Starting background tasks...")
        self.runner = web.AppRunner(self.app)
        await self.runner.setup()
        self.site = web.TCPSite(self.runner, host, port)
        await self.site.start()
        logger.info(f"🚀 WebServer started at http://{host}:{port}")

    async def stop_background(self):
        print("Stopping background tasks...")
        logger.info("🛑 Stopping WebServer...")
        await self.cleanup(self.app)
        await self.runner.cleanup()
        logger.info("✅ WebServer stopped.")


if __name__ == '__main__':
    try:
        webserver = WebServer()
        webserver.app.on_shutdown.append(webserver.cleanup)
        web.run_app(webserver.app, port=4664)

    except KeyboardInterrupt:
        logger.info("🛑 KeyboardInterrupt received. Exiting...")
