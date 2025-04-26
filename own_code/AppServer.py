import asyncio
import json
from aiohttp import web
from aiortc import RTCPeerConnection, RTCSessionDescription, RTCDataChannel
from threading import Thread, Lock
import gi
gi.require_version('Gst', '1.0')
gi.require_version('GstWebRTC', '1.0')
from gi.repository import Gst, GstWebRTC, GObject, GstSdp
from detection_engine import DetectionEngine

detector = DetectionEngine(model_path='/home/pi/Adeept_DarkPaw/own_code/models/yolov11m.hef',
                           score_thresh=0.65,
                           max_detections=3)

command_queue = asyncio.Queue()
data_queue = asyncio.Queue()

# Initialize GStreamer
Gst.init(None)

pcs = set()  # Peer connections
picam2 = detector.camera  # Global camera instance
camera_lock = Lock()

frame_count = 0


# === Web Routes ===
async def index(request):
    return web.FileResponse('./templates/index.html')


async def javascript(request):
    return web.FileResponse('./static/scripts.js')


async def jquery(request):
    return web.FileResponse('./static/jquery-3.2.1.min.js')


async def styles(request):
    return web.FileResponse('./static/style.css')


# Simulated data source
async def fake_data_updater():
    import random
    while True:
        await asyncio.sleep(0.1)
        data = {
            "Distance": f"{random.randint(20, 100)}",
            "CPU_temp": f"{random.uniform(40.0, 60.0):.1f}",
            "CPU_load": f"{random.uniform(10.0, 90.0):.1f}",
            "RAM_usage": f"{random.uniform(20.0, 80.0):.1f}"
        }
        await data_queue.put(data)


# === WebSocket/WebRTC handler ===
async def websocket_handler(request):
    global picam2

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
    if not camera_lock.acquire(blocking=False):
        print("❌ Camera already in use, rejecting connection.")
        return web.Response(text="Camera busy", status=503)

    def feed_frame(request):
        global frame_count
        frame = request.make_array("main")
        frame_with_detections = detector.postprocess_frames(frame)
        data = frame_with_detections.tobytes()
        buf = Gst.Buffer.new_allocate(None, len(data), None)
        buf.fill(0, data)

        # Assuming 30 fps
        buf.pts = buf.dts = int(frame_count * Gst.SECOND / 30)
        buf.duration = int(Gst.SECOND / 30)
        frame_count += 1

        ret = src.emit("push-buffer", buf)
        if ret != Gst.FlowReturn.OK:
            print("❌ Failed to push buffer into GStreamer:", ret)

    camera_config = picam2.create_video_configuration(
        main={'size': (800, 600), 'format': 'XRGB8888'},
        controls={'FrameRate': 30}
    )
    picam2.configure(camera_config)
    picam2.pre_callback = feed_frame
    picam2.start()

    # === WebRTC setup ===
    pcs.add(ws)

    def on_negotiation_needed(element):
        print("Negotiation needed")
        promise = Gst.Promise.new_with_change_func(on_offer_created, ws, None)
        webrtc.emit('create-offer', None, promise)

    def on_offer_created(promise, ws_conn, _user_data):
        print("Offer created")
        reply = promise.get_reply()
        offer = reply.get_value("offer")
        webrtc.emit('set-local-description', offer, None)

        sdp_msg = json.dumps({'sdp': {
            'type': 'offer',
            'sdp': offer.sdp.as_text()
        }})
        asyncio.run_coroutine_threadsafe(ws.send_str(sdp_msg), loop)

    def on_ice_candidate(_, mlineindex, candidate):
        print("Python sending ICE:", candidate)
        ice_msg = json.dumps({'ice': {
            'candidate': candidate,
            'sdpMLineIndex': mlineindex,
        }})
        asyncio.run_coroutine_threadsafe(ws.send_str(ice_msg), loop)

    webrtc.connect('on-negotiation-needed', on_negotiation_needed)
    webrtc.connect('on-ice-candidate', on_ice_candidate)

    def on_data_channel(element, data_channel):
        print("📡 Data channel received")

        @data_channel.connect("on-open")
        def on_open(channel):
            print("✅ Data channel is now open and ready to use")

        @data_channel.connect("on-message-string")
        def on_message(channel, message):
            print("📥 Received message on data channel:", message)

            if message == "request_status":
                # Return latest status data
                async def send_status():
                    if not data_queue.empty():
                        data = await data_queue.get()
                        data["type"] = "status_update"
                        json_data = json.dumps(data)
                        channel.send(json_data)

                asyncio.run_coroutine_threadsafe(send_status(), loop)

            else:
                # Any command is just put into queue
                command_queue.put_nowait(message)
                print("✅ Command queued:", message)

    # Connect to the signal
    webrtc.connect("on-data-channel", on_data_channel)

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

                elif 'ice' in data:
                    ice = data['ice']
                    webrtc.emit('add-ice-candidate', ice['sdpMLineIndex'], ice['candidate'])

    except Exception as e:
        print("WebSocket error:", e)

    # Cleanup
    print("🛑 Cleaning up...")
    try:
        pipeline.set_state(Gst.State.NULL)
        print("🛑 Pipeline stopped.")
    except Exception as e:
        print("❌ Failed to stop pipeline:", e)

    try:
        picam2.stop()
        print("📷 Picamera2 stopped.")
    except Exception as e:
        print("❌ Failed to stop Picamera2:", e)

    camera_lock.release()
    return ws


# === App setup ===

app = web.Application()
app.router.add_get('/', index)
app.router.add_get('/static/scripts.js', javascript)
app.router.add_get('/static/jquery-3.2.1.min.js', jquery)
app.router.add_get('/static/style.css', styles)
app.router.add_get('/ws', websocket_handler)

detection_thread = Thread(target=detector.run_forever)
detection_thread.start()

web.run_app(app, port=4664)

# import logging
#
# # Import necessary libraries
# from queue import Queue
# from flask import Flask, render_template, Response, send_from_directory
# from werkzeug.serving import make_server
# import sys
# # import io
# # import socketserver
# from threading import Condition, Thread, Event
# # from http import server
# from picamera2 import MappedArray, Picamera2
# # from picamera2.encoders import JpegEncoder
# # from picamera2.outputs import FileOutput
# # from libcamera import controls
# import asyncio
# import json
# from aiohttp import web
# import random
# import asyncio
# import time
# from webrtc_sendrecv import WebRTCClient, check_plugins, PIPELINE_DESC
# import gi
# gi.require_version('Gst', '1.0')
# from gi.repository import Gst
# gi.require_version('GstWebRTC', '1.0')
# from gi.repository import GstWebRTC
# gi.require_version('GstSdp', '1.0')
# from gi.repository import GstSdp
#
# logging.basicConfig(level=logging.INFO)
# logger = logging.getLogger(__name__)
#
# # for triggering the shutdown procedure when a signal is detected
# keyboard_trigger = Event()
#
#
# def signal_handler(signal, frame):
#     logging.info('Signal detected. Stopping threads.')
#     keyboard_trigger.set()
#
#
# #######################
# #   Web Server Stuff  #
# #######################
# class WebServerThread(Thread):
#     """
#     Class to make the launch of the flask server non-blocking.
#     Also adds shutdown functionality to it.
#     """
#     def __init__(self, command_q: Queue, data_q: Queue, host="0.0.0.0", port=4664,
#                  directory_path='/home/pi/Adeept_DarkPaw/own_code/static/'):
#         Thread.__init__(self)
#         self.app = Flask(__name__, static_url_path='')
#         self.srv = make_server(host, port, self.app)
#         self.ctx = self.app.app_context()
#         self.ctx.push()
#         self.data_dict = {'Distance': "N/A", 'CPU_temp': "N/A", 'CPU_load': "N/A", 'RAM_usage': "N/A"}
#         self.cmd_queue = command_q
#         self.data_q = data_q
#         self.directory_path = directory_path
#         self.register_endpoints()
#
#     def register_endpoints(self):
#         self.app.add_url_rule("/", view_func=self.index)
#         self.app.add_url_rule("/index", view_func=self.index)
#         self.app.add_url_rule("/<string:page_name>", view_func=self.page)
#         self.app.add_url_rule("/static/<path:path>", view_func=self.send_static)
#         self.app.add_url_rule("/process_button_click/<command_string>", view_func=self.process_button_click)
#         self.app.add_url_rule("/process_velocity_change/<command_string>", view_func=self.process_vel_change)
#         self.app.add_url_rule("/process_mode_change/<command_string>", view_func=self.process_mode_change)
#         self.app.add_url_rule("/read_data", view_func=self.send_data)
#
#     @staticmethod
#     def index():
#         # return the rendered template
#         return render_template("index.html")
#
#     def process_button_click(self, command_string):
#         self.cmd_queue.put(command_string)
#         return Response()
#
#     def process_vel_change(self, command_string):
#         self.cmd_queue.put('velocity_' + command_string)
#         return Response()
#
#     def process_mode_change(self, command_string):
#         self.cmd_queue.put('mode_select:' + command_string)
#         return Response()
#
#     @staticmethod
#     def page(page_name):
#         return render_template("{}".format(page_name))
#
#     def send_static(self, path):
#         return send_from_directory(self.directory_path, path)
#
#     def send_data(self):
#         if not self.data_q.empty():
#             self.data_dict = self.data_q.get()
#         return self.data_dict
#
#     def run(self):
#         logging.info('Starting Flask server')
#         self.srv.serve_forever()
#
#     def shutdown(self):
#         logging.info('Stopping Flask server')
#         self.srv.shutdown()
#
#
# def setup_webserver(command_q: Queue, data_q: Queue, pipe_reference,
#                     host="127.0.0.1", port=4664, video_port=4665):
#     # registering both types of signals
#     # videostream = StreamingServer((host, video_port), StreamingHandler)
#     # camera_instance.start_recording(JpegEncoder(), FileOutput(StreamingHandler.output))
#     if not check_plugins():
#         sys.exit(1)
#     our_id = random.randrange(10, 10000)
#     c = WebRTCClient(our_id, "client", "ws://" + host + ":" + str(video_port), pipe_reference)
#     loop = asyncio.get_event_loop()
#     loop.run_until_complete(c.connect())
#     # res = loop.run_until_complete(c.loop())
#
#     stream_server = Thread(target=loop.run_until_complete, args=[c.loop()])
#     stream_server.start()
#     logging.info("Started stream server for picamera2")
#
#     # starting the web server
#     web_server = WebServerThread(command_q, data_q, host=host, port=port)
#     web_server.start()
#     logging.info("Started Flask web server")
#
#     return stream_server, web_server
#
#
# if __name__ == '__main__':
#     command_queue = Queue()
#     data_queue = Queue()
#     # ToDo: Test Code!!!
#     # camera = Picamera2()
#     # video_w, video_h = 800, 600
#     # camera.set_controls({"AwbMode": controls.AwbModeEnum.Indoor})
#     # camera_config = camera.create_video_configuration(main={'size': (video_w, video_h), 'format': 'XRGB8888'},
#     #                                                   raw={'format': 'SGRBG10'}, controls={'FrameRate': 30})
#     # camera.preview_configuration.align()
#     # camera.configure(camera_config)
#     Gst.init(None)
#     pipe = Gst.parse_launch(PIPELINE_DESC)
#     streamserver, webserver = setup_webserver(command_queue, data_queue, pipe)
#
#     # and run it indefinitely
#     while not keyboard_trigger.is_set():
#         time.sleep(0.5)
#
#     # until some keyboard event is detected
#     logging.info("Keyboard event detected")
#
#     # trigger shutdown procedure
#     webserver.shutdown()
#
#     # and finalize shutting them down
#     webserver.join()
#     streamserver.join()
#     logging.info("Stopped all threads")
