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

        logger.info("🌐 WebRTC client connected")

        # ================================
        # Create pipeline
        # ================================
        pipeline = Gst.Pipeline.new("webrtc-pipeline")

        src = Gst.ElementFactory.make("appsrc", "source")
        conv = Gst.ElementFactory.make("videoconvert", "convert")
        enc = Gst.ElementFactory.make("vp8enc", "encoder")
        pay = Gst.ElementFactory.make("rtpvp8pay", "pay")
        webrtc = Gst.ElementFactory.make("webrtcbin", "webrtc")

        for e in [src, conv, enc, pay, webrtc]:
            pipeline.add(e)

        src.link(conv)
        conv.link(enc)
        enc.link(pay)

        # ================================
        # appsrc config
        # ================================
        src.set_property("is-live", True)
        src.set_property("format", Gst.Format.TIME)
        src.set_property("block", True)
        src.set_property(
            "caps",
            Gst.Caps.from_string(
                "video/x-raw,format=BGRx,width=800,height=600,framerate=30/1"
            ),
        )

        enc.set_property("deadline", 1)
        enc.set_property("target-bitrate", 1_000_000)

        # IMPORTANT: set payload type
        pay.set_property("pt", 96)

        # --- Add transceiver ---
        caps = Gst.Caps.from_string(
            "application/x-rtp,media=video,encoding-name=VP8,payload=96"
        )

        webrtc.emit(
            "add-transceiver",
            GstWebRTC.WebRTCRTPTransceiverDirection.SENDONLY,
            caps,
        )

        logger.info("✅ Transceiver added")

        # --- Link dynamically when pad appears ---
        def on_webrtc_pad_added(element, pad):
            logger.info(f"🧩 webrtcbin pad added: {pad.get_name()}")

            if pad.get_direction() != Gst.PadDirection.SINK:
                return

            src_pad = pay.get_static_pad("src")
            if not src_pad:
                logger.error("❌ Payloader src pad missing")
                return

            if src_pad.is_linked():
                logger.info("⚠️ Payloader already linked")
                return

            ret = src_pad.link(pad)
            if ret == Gst.PadLinkReturn.OK:
                logger.info("✅ Payloader linked to webrtcbin")
            else:
                logger.error(f"❌ Link failed: {ret}")

        webrtc.connect("pad-added", on_webrtc_pad_added)

        # ================================
        # Data channel handling
        # ================================
        def on_data_channel(_, channel):
            logger.info(f"📡 Data channel received: {channel.get_label()}")

            def on_open(_):
                logger.info("✅ Data channel open")

            def on_message(_, msg):
                logger.info(f"📥 Command: {msg}")

                if self.command_queue.full():
                    self.command_queue.get_nowait()

                self.command_queue.put_nowait(msg)

            channel.connect("on-open", on_open)
            channel.connect("on-message-string", on_message)

        webrtc.connect("on-data-channel", on_data_channel)

        # Create server channel
        # webrtc.emit("create-data-channel", "control", None)
        logger.info("📡 Server data channel created")

        # ================================
        # Negotiation
        # ================================
        def on_negotiation_needed(element):
            logger.info("🧠 Negotiation needed")

            promise = Gst.Promise.new_with_change_func(
                on_offer_created, element, None
            )
            element.emit("create-offer", None, promise)

        def on_offer_created(promise, element, _):

            channel = webrtc.emit("create-data-channel", "control", None, )

            if channel:
                logger.info("📡 Server data channel created")

            reply = promise.get_reply()
            offer = reply.get_value("offer")

            element.emit("set-local-description", offer, None)

            text = offer.sdp.as_text()

            msg = json.dumps({"sdp": {"type": "offer", "sdp": text}})

            asyncio.run_coroutine_threadsafe(ws.send_str(msg), loop)

            # Mark pipeline ready ONLY after offer
            self.pipeline_ready = True
            logger.info("🚀 Pipeline ready for frames")

        webrtc.connect(
            "on-negotiation-needed", on_negotiation_needed
        )

        # ================================
        # ICE
        # ================================
        def on_ice_candidate(_, mline, candidate):
            logger.info(f"🧊 ICE → {candidate}")
            msg = json.dumps(
                {"ice": {
                        "candidate": candidate,
                        "sdpMLineIndex": mline,}
                })

            asyncio.run_coroutine_threadsafe(ws.send_str(msg), loop)

        webrtc.connect("on-ice-candidate", on_ice_candidate)

        # ================================
        # Start pipeline
        # ================================
        pipeline.set_state(Gst.State.PLAYING)
        logger.info("🎬 Pipeline set to PLAYING")
        # Force negotiation once pipeline is running
        GObject.idle_add(
            lambda: webrtc.emit("on-negotiation-needed")
        )

        # ================================
        # Frame pusher
        # ================================
        def frame_pusher():
            while True:
                frame = self.frame_queue.get()
                if not self.pipeline_ready:
                    continue
                frame = self.detector.postprocess_frames(frame)
                data = frame.tobytes()

                buf = Gst.Buffer.new_allocate(None, len(data), None)
                buf.fill(0, data)
                buf.pts = buf.dts = int(
                    self.frame_count * Gst.SECOND / 30
                )
                buf.duration = int(Gst.SECOND / 30)

                self.frame_count += 1

                ret = src.emit("push-buffer", buf)

                if ret != Gst.FlowReturn.OK:
                    logger.warning(f"Push failed: {ret}")

        Thread(target=frame_pusher, daemon=True).start()

        # ================================
        # Camera start
        # ================================
        if not self.camera_lock.acquire(False):
            return web.Response(
                text="Camera busy", status=503
            )

        def feed_frame(req):
            frame = req.make_array("main")

            if self.frame_queue.full():
                self.frame_queue.get_nowait()

            self.frame_queue.put_nowait(frame)

        self.picam2.pre_callback = feed_frame
        self.picam2.start()

        logger.info("📷 Camera started")

        # ================================
        # Signaling receive loop
        # ================================
        async for msg in ws:

            if msg.type == web.WSMsgType.TEXT:
                data = json.loads(msg.data)

                if "sdp" in data:
                    logger.info("📨 Answer received")

                    sdp = data["sdp"]

                    res, sdpmsg = (
                        GstSdp.SDPMessage.new_from_text(
                            sdp["sdp"]
                        )
                    )

                    answer = (
                        GstWebRTC.WebRTCSessionDescription.new(
                            GstWebRTC.WebRTCSDPType.ANSWER,
                            sdpmsg,
                        )
                    )

                    webrtc.emit(
                        "set-remote-description",
                        answer,
                        None,
                    )

                elif "ice" in data:
                    ice = data["ice"]

                    webrtc.emit(
                        "add-ice-candidate",
                        ice["sdpMLineIndex"],
                        ice["candidate"],
                    )

        # ================================
        # Cleanup
        # ================================
        logger.info("🛑 Cleaning up")

        pipeline.set_state(Gst.State.NULL)
        self.stop_camera()

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
