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

data_channel_set_up = False
negotiation_needed = False

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

    def setup_data_channel():
        global data_channel_set_up
        if data_channel_set_up:
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
                async def send_status():
                    if not data_queue.empty():
                        data = await data_queue.get()
                        data["type"] = "status_update"
                        json_data = json.dumps(data)
                        channel.send(json_data)

                asyncio.run_coroutine_threadsafe(send_status(), loop)
            else:
                command_queue.put_nowait(message)
                print("✅ Command queued:", message)

        data_channel.connect("on-open", on_open)
        data_channel.connect("on-message-string", on_message)

        data_channel_set_up = True  # Mark the data channel as set up

    def on_negotiation_needed(element):
        global negotiation_needed
        if not negotiation_needed:
            print("Negotiation needed, starting negotiation...")
            negotiation_needed = True
            promise = Gst.Promise.new_with_change_func(on_offer_created, ws, None)
            webrtc.emit('create-offer', None, promise)
        else:
            print("Skipping negotiation because one is already in progress.")

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

        # Reset negotiation flag after the offer is created
        global negotiation_needed
        negotiation_needed = False

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