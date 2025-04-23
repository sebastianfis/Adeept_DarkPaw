import asyncio
import json
from aiohttp import web
import gi
gi.require_version('Gst', '1.0')
gi.require_version('GstWebRTC', '1.0')
from gi.repository import Gst, GstWebRTC, GObject, GstSdp

from picamera2 import Picamera2
import numpy as np

# Initialize GStreamer
Gst.init(None)

pcs = set()  # Peer connections
picam2 = None  # Global camera instance

frame_count = 0

# === Web Routes ===

async def index(request):
    return web.FileResponse('./static/minimal_index.html')


async def javascript(request):
    return web.FileResponse('./static/video_client.js')


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
        "video/x-raw,format=I420,width=1280,height=720,framerate=30/1"))

    # Other element properties
    # caps.set_property("caps", Gst.Caps.from_string("video/x-raw,width=640,height=480,framerate=30/1"))
    encoder.set_property("deadline", 1)

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
    picam2 = Picamera2()

    def feed_frame(request):
        global frame_count
        frame = request.make_array("main")
        data = frame.tobytes()
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
        main={'size': (640, 480), 'format': 'XRGB8888'},
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
    pipeline.set_state(Gst.State.NULL)
    picam2.stop()
    return ws


# === App setup ===

app = web.Application()
app.router.add_get('/', index)
app.router.add_get('/static/video_client.js', javascript)
app.router.add_get('/ws', websocket_handler)

web.run_app(app, port=4664)