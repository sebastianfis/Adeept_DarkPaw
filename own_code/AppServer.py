import logging

# Import necessary libraries
from queue import Queue
from flask import Flask, render_template, Response, send_from_directory
from werkzeug.serving import make_server
import sys
# import io
# import socketserver
from threading import Condition, Thread, Event
# from http import server
from picamera2 import MappedArray, Picamera2
# from picamera2.encoders import JpegEncoder
# from picamera2.outputs import FileOutput
# from libcamera import controls
import asyncio
import json
from aiohttp import web
import random
import asyncio
import time
from webrtc_sendrecv import WebRTCClient, check_plugins, PIPELINE_DESC
import gi
gi.require_version('Gst', '1.0')
from gi.repository import Gst
gi.require_version('GstWebRTC', '1.0')
from gi.repository import GstWebRTC
gi.require_version('GstSdp', '1.0')
from gi.repository import GstSdp

logging.basicConfig(level=logging.INFO)
logger = logging.getLogger(__name__)

# for triggering the shutdown procedure when a signal is detected
keyboard_trigger = Event()


def signal_handler(signal, frame):
    logging.info('Signal detected. Stopping threads.')
    keyboard_trigger.set()


#######################
#   Web Server Stuff  #
#######################
class WebServerThread(Thread):
    """
    Class to make the launch of the flask server non-blocking.
    Also adds shutdown functionality to it.
    """
    def __init__(self, command_q: Queue, data_q: Queue, host="0.0.0.0", port=4664,
                 directory_path='/home/pi/Adeept_DarkPaw/own_code/static/'):
        Thread.__init__(self)
        self.app = Flask(__name__, static_url_path='')
        self.srv = make_server(host, port, self.app)
        self.ctx = self.app.app_context()
        self.ctx.push()
        self.data_dict = {'Distance': "N/A", 'CPU_temp': "N/A", 'CPU_load': "N/A", 'RAM_usage': "N/A"}
        self.cmd_queue = command_q
        self.data_q = data_q
        self.directory_path = directory_path
        self.register_endpoints()

    def register_endpoints(self):
        self.app.add_url_rule("/", view_func=self.index)
        self.app.add_url_rule("/index", view_func=self.index)
        self.app.add_url_rule("/<string:page_name>", view_func=self.page)
        self.app.add_url_rule("/static/<path:path>", view_func=self.send_static)
        self.app.add_url_rule("/process_button_click/<command_string>", view_func=self.process_button_click)
        self.app.add_url_rule("/process_velocity_change/<command_string>", view_func=self.process_vel_change)
        self.app.add_url_rule("/process_mode_change/<command_string>", view_func=self.process_mode_change)
        self.app.add_url_rule("/read_data", view_func=self.send_data)

    @staticmethod
    def index():
        # return the rendered template
        return render_template("index.html")

    def process_button_click(self, command_string):
        self.cmd_queue.put(command_string)
        return Response()

    def process_vel_change(self, command_string):
        self.cmd_queue.put('velocity_' + command_string)
        return Response()

    def process_mode_change(self, command_string):
        self.cmd_queue.put('mode_select:' + command_string)
        return Response()

    @staticmethod
    def page(page_name):
        return render_template("{}".format(page_name))

    def send_static(self, path):
        return send_from_directory(self.directory_path, path)

    def send_data(self):
        if not self.data_q.empty():
            self.data_dict = self.data_q.get()
        return self.data_dict

    def run(self):
        logging.info('Starting Flask server')
        self.srv.serve_forever()

    def shutdown(self):
        logging.info('Stopping Flask server')
        self.srv.shutdown()


#############################
#   Video Streaming Stuff   #
#############################
pcs = set()


async def index(request):
    return web.FileResponse('./index.html')


async def javascript(request):
    return web.FileResponse('./client.js')


async def websocket_handler(request):
    ws = web.WebSocketResponse()
    await ws.prepare(request)

    pipeline = Gst.parse_launch(
        'videotestsrc is-live=true ! videoconvert ! vp8enc deadline=1 ! rtpvp8pay ! webrtcbin name=sendrecv'
    )
    webrtc = pipeline.get_by_name('sendrecv')

    pcs.add(ws)

    def on_negotiation_needed(element):
        offer = webrtc.emit('create-offer', None)
        offer.connect('done', lambda src, promise: on_offer_created(src, promise, ws))

    def on_offer_created(src, promise, ws_conn):
        reply = src.emit('create-offer-done', promise)
        webrtc.emit('set-local-description', reply, None)
        text = json.dumps({'sdp': {
            'type': 'offer',
            'sdp': reply.sdp.as_text()
        }})
        asyncio.run_coroutine_threadsafe(ws_conn.send_str(text), asyncio.get_event_loop())

    def on_ice_candidate(_, mlineindex, candidate):
        ice = json.dumps({'ice': {
            'candidate': candidate,
            'sdpMLineIndex': mlineindex,
        }})
        asyncio.run_coroutine_threadsafe(ws.send_str(ice), asyncio.get_event_loop())

    webrtc.connect('on-negotiation-needed', on_negotiation_needed)
    webrtc.connect('on-ice-candidate', on_ice_candidate)

    pipeline.set_state(Gst.State.PLAYING)

    async for msg in ws:
        if msg.type == web.WSMsgType.TEXT:
            data = json.loads(msg.data)

            if 'sdp' in data:
                sdp = data['sdp']
                desc = GstWebRTC.WebRTCSessionDescription.new(
                    GstWebRTC.WebRTCSDPType.ANSWER if sdp['type'] == 'answer' else GstWebRTC.WebRTCSDPType.OFFER,
                    Gst.SDPMessage.new_from_text(sdp['sdp'])
                )
                webrtc.emit('set-remote-description', desc, None)
            elif 'ice' in data:
                ice = data['ice']
                webrtc.emit('add-ice-candidate', ice['sdpMLineIndex'], ice['candidate'])

    pipeline.set_state(Gst.State.NULL)
    return ws

#
# class StreamingOutput(io.BufferedIOBase):
#     def __init__(self):
#         self.frame = None
#         self.condition = Condition()
#
#     def write(self, buf):
#         with self.condition:
#             self.frame = buf
#             self.condition.notify_all()
#
#
# class StreamingHandler(server.BaseHTTPRequestHandler):
#     """
#     Implementing GET request for the video stream.
#     """
#     output = StreamingOutput()
#
#     def __init__(self, request: bytes, client_address: tuple[str, int],
#                  server: socketserver.BaseServer):
#         super().__init__(request, client_address, server)
#
#     def do_GET(self):
#         if self.path == '/stream.mjpg':
#             self.send_response(200)
#             self.send_header('Age', '0')
#             self.send_header('Cache-Control', 'no-cache, private')
#             self.send_header('Pragma', 'no-cache')
#             self.send_header('Content-Type', 'multipart/x-mixed-replace; boundary=FRAME')
#             self.end_headers()
#             try:
#                 while True:
#                     with self.output.condition:
#                         self.output.condition.wait()
#                         frame = self.output.frame
#                     self.wfile.write(b'--FRAME\r\n')
#                     self.send_header('Content-Type', 'image/jpeg')
#                     self.send_header('Content-Length', len(frame))
#                     self.end_headers()
#                     self.wfile.write(frame)
#                     self.wfile.write(b'\r\n')
#             except Exception as e:
#                 logging.warning(
#                     'Removed streaming client %s: %s',
#                     self.client_address, str(e))
#         else:
#             self.send_error(404)
#             self.end_headers()
#
#
# class StreamingServer(socketserver.ThreadingMixIn, server.HTTPServer):
#     allow_reuse_address = True
#     daemon_threads = True


def setup_webserver(command_q: Queue, data_q: Queue, pipe_reference,
                    host="127.0.0.1", port=4664, video_port=4665):
    # registering both types of signals
    # videostream = StreamingServer((host, video_port), StreamingHandler)
    # camera_instance.start_recording(JpegEncoder(), FileOutput(StreamingHandler.output))
    if not check_plugins():
        sys.exit(1)
    our_id = random.randrange(10, 10000)
    c = WebRTCClient(our_id, "client", "ws://" + host + ":" + str(video_port), pipe_reference)
    loop = asyncio.get_event_loop()
    loop.run_until_complete(c.connect())
    # res = loop.run_until_complete(c.loop())

    stream_server = Thread(target=loop.run_until_complete, args=[c.loop()])
    stream_server.start()
    logging.info("Started stream server for picamera2")

    # starting the web server
    web_server = WebServerThread(command_q, data_q, host=host, port=port)
    web_server.start()
    logging.info("Started Flask web server")

    return stream_server, web_server


if __name__ == '__main__':
    command_queue = Queue()
    data_queue = Queue()
    # ToDo: Test Code!!!    # camera = Picamera2()
    # video_w, video_h = 800, 600
    # camera.set_controls({"AwbMode": controls.AwbModeEnum.Indoor})
    # camera_config = camera.create_video_configuration(main={'size': (video_w, video_h), 'format': 'XRGB8888'},
    #                                                   raw={'format': 'SGRBG10'}, controls={'FrameRate': 30})
    # camera.preview_configuration.align()
    # camera.configure(camera_config)
    Gst.init(None)
    pipe = Gst.parse_launch(PIPELINE_DESC)
    streamserver, webserver = setup_webserver(command_queue, data_queue, pipe)

    # and run it indefinitely
    while not keyboard_trigger.is_set():
        time.sleep(0.5)

    # until some keyboard event is detected
    logging.info("Keyboard event detected")

    # trigger shutdown procedure
    webserver.shutdown()

    # and finalize shutting them down
    webserver.join()
    streamserver.join()
    logging.info("Stopped all threads")
