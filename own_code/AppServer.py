import asyncio
import json
from aiohttp import web
import logging
from threading import Thread, Lock
import gi
import time, queue
gi.require_version('Gst', '1.0')
gi.require_version('GstWebRTC', '1.0')
from gi.repository import Gst, GstWebRTC, GObject, GstSdp
from detection_engine import DetectionEngine

# ToDo: Write wrapper class around this whole mess!!!


logging.basicConfig(level=logging.INFO)
logger = logging.getLogger(__name__)

class WebServer:
    def __init__(self, detector: DetectionEngine):
        self.command_queue = queue.Queue(maxsize=1)
        self.data_queue = queue.Queue()
        self.frame_queue = queue.Queue(maxsize=5)  # Keep it small to avoid latency
        self.pcs = set()  # Peer connections
        self.picam2 = detector.camera  # Global camera instance
        self.detector = detector
        self.camera_lock = Lock()
        self.frame_count = 0
        self.data_channel_set_up = False
        self.negotiation_in_progress = False

        self.measurement_thread = Thread(target=self.fake_data_updater)
        self.measurement_thread.start()

        self.detection_thread = Thread(target=self.detector.run_forever)
        self.detection_thread.start()

        # Initialize GStreamer
        Gst.init(None)

        # === App setup ===

        self.app = web.Application()
        self.app.router.add_get('/', self.index)
        self.app.router.add_get('/static/scripts.js', self.javascript)
        self.app.router.add_get('/static/jquery-3.2.1.min.js', self.jquery)
        self.app.router.add_get('/static/style.css', self.styles)
        self.app.router.add_get('/ws', self.websocket_handler)

        web.run_app(self.app, port=4664)

    # Simulated data source
    def fake_data_updater(self):
        import random
        while True:
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
            print("❌ Failed to link payloader to webrtcbin")
        else:
            print("✅ Linked payloader to webrtcbin")

        # === Picamera2 setup ===
        if not self.camera_lock.acquire(blocking=False):
            print("❌ Camera already in use, rejecting connection.")
            return web.Response(text="Camera busy", status=503)

        def feed_frame(request):
            frame = request.make_array("main")
            if self.frame_queue.full():
                try:
                    self.frame_queue.get_nowait()  # Drop the oldest frame to prevent queue backup
                except queue.Empty:
                    pass
            self.frame_queue.put_nowait(frame)

        def frame_pusher():
            while True:
                try:
                    frame = self.frame_queue.get(timeout=1)  # Wait max 1 sec for a frame
                except queue.Empty:
                    continue  # No frame, just loop

                # (Postprocess your frame here)
                frame_with_detections = self.detector.postprocess_frames(frame)
                data = frame_with_detections.tobytes()

                buf = Gst.Buffer.new_allocate(None, len(data), None)
                buf.fill(0, data)

                buf.pts = buf.dts = int(self.frame_count * Gst.SECOND / 30)
                buf.duration = int(Gst.SECOND / 30)
                self.frame_count += 1

                ret = src.emit("push-buffer", buf)
                if ret != Gst.FlowReturn.OK:
                    print("❌ Failed to push buffer into GStreamer:", ret)

        pusher_thread = Thread(target=frame_pusher, daemon=True)
        pusher_thread.start()

        camera_config = self.picam2.create_video_configuration(
            main={'size': (800, 600), 'format': 'XRGB8888'},
            controls={'FrameRate': 30}
        )
        self.picam2.configure(camera_config)
        self.picam2.pre_callback = feed_frame
        self.picam2.start()

        # === WebRTC setup ===
        self.pcs.add(ws)

        def setup_data_channel():
            if self.data_channel_set_up:
                print("❌ Data channel already set up!")
                return  # Prevent re-setup of the data channel

            data_channel = webrtc.emit("create-data-channel", "control", None)
            if not data_channel:
                print("❌ Could not create data channel!")
                return
            print("📡 Server created data channel")

            def on_open(channel):
                print("✅ Server data channel is now open")

            def on_message(channel, message):
                print("📥 Received message on data channel:", message)

                if message == "request_status":
                    if not self.data_queue.empty():
                        data = self.data_queue.get()
                        data["type"] = "status_update"
                        json_data = json.dumps(data)
                        print("✅ Sending message:", json_data)
                        channel.emit("send-string", json_data)  # <-- send directly!
                    else:
                        print("⚠ Data queue empty, nothing to send.")
                else:
                    if self.command_queue.full():
                        try:
                            self.command_queue.get_nowait()  # Drop the oldest frame to prevent queue backup
                        except queue.Empty:
                            pass
                    self.command_queue.put_nowait(message)
                    print("✅ Command queued:", message)

            data_channel.connect("on-open", on_open)
            data_channel.connect("on-message-string", on_message)
            self.data_channel_set_up = True  # Mark the data channel as set up

        def on_negotiation_needed(element):
            # Check if a negotiation is already in progress
            if self.negotiation_in_progress:
                print("❌ Negotiation already in progress, skipping offer creation.")
                return

            print("Negotiation needed")
            self.negotiation_in_progress = True  # Set the flag to indicate negotiation is in progress

            # Create a promise and handle the offer creation
            promise = Gst.Promise.new_with_change_func(on_offer_created, ws, None)
            webrtc.emit('create-offer', None, promise)

        def on_offer_created(promise, ws_conn, _user_data):
            print("Offer created")
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
            print("Python sending ICE:", candidate)
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
                            print("❌ Failed to parse SDP answer")
                            return
                        answer = GstWebRTC.WebRTCSessionDescription.new(GstWebRTC.WebRTCSDPType.ANSWER, sdpmsg)
                        webrtc.emit('set-remote-description', answer, None)

                        # Only set up the data channel once
                        setup_data_channel()

                    elif 'ice' in data:
                        ice = data['ice']
                        webrtc.emit('add-ice-candidate', ice['sdpMLineIndex'], ice['candidate'])

        except Exception as e:
            print("WebSocket error:", e)

        except Exception as e:
            print("WebSocket error:", e)

        finally:
            # This runs both after an exception OR after clean disconnection
            print("🛑 Cleaning up...")
            try:
                pipeline.set_state(Gst.State.NULL)
                print("🛑 Pipeline stopped.")
            except Exception as e:
                print("❌ Failed to stop pipeline:", e)

            try:
                self.picam2.stop()
                print("📷 Picamera2 stopped.")
            except Exception as e:
                print("❌ Failed to stop Picamera2:", e)

            self.camera_lock.release()

            # ✅ Reset your flags here
            self.data_channel_set_up = False
            print("🔄 Reset data_channel_set_up flag")

        if self.camera_lock.locked():
            self.camera_lock.release()
        return ws


if __name__ == '__main__':
    det = DetectionEngine(model_path='/home/pi/Adeept_DarkPaw/own_code/models/yolov11m.hef',
                          score_thresh=0.65,
                          max_detections=3)
    webserver = WebServer(det)
