import asyncio
import json
from aiohttp import web
import gi
gi.require_version('Gst', '1.0')
gi.require_version('GstWebRTC', '1.0')
from gi.repository import Gst, GstWebRTC, GObject

Gst.init(None)

pcs = set()

loop = asyncio.get_event_loop()

async def index(request):
    return web.FileResponse('./static/minimal_index.html')


async def javascript(request):
    return web.FileResponse('./static/video_client.js')



async def websocket_handler(request):
    ws = web.WebSocketResponse()
    await ws.prepare(request)

    pipeline = Gst.Pipeline.new("webrtc-pipeline")

    # Create elements
    src = Gst.ElementFactory.make("videotestsrc", "source")
    conv = Gst.ElementFactory.make("videoconvert", "convert")
    scale = Gst.ElementFactory.make("videoscale", "scale")
    caps = Gst.ElementFactory.make("capsfilter", "caps")
    encoder = Gst.ElementFactory.make("vp8enc", "encoder")
    payloader = Gst.ElementFactory.make("rtpvp8pay", "pay")
    webrtc = Gst.ElementFactory.make("webrtcbin", "sendrecv")

    # Set element properties
    src.set_property("is-live", True)
    caps.set_property("caps", Gst.Caps.from_string("video/x-raw,width=640,height=480,framerate=30/1"))
    encoder.set_property("deadline", 1)

    # Add elements to pipeline
    for elem in [src, conv, scale, caps, encoder, payloader, webrtc]:
        pipeline.add(elem)

    # Link static pads
    src.link(conv)
    conv.link(scale)
    scale.link(caps)
    caps.link(encoder)
    encoder.link(payloader)
    payloader.link(webrtc)  # <- Direct static link

    # Link payloader dynamically to webrtcbin
    # def on_pad_added(element, pad):
    #     print(f" Pad added from {element.get_name()}: {pad.get_name()}")
    #     sinkpad = webrtc.get_request_pad("sink_0")
    #     if sinkpad:
    #         pad.link(sinkpad)
    #
    # payloader.connect("pad-added", on_pad_added)
    #
    # # Add caps to RTP stream
    # payloader.link_filtered(webrtc, Gst.Caps.from_string("application/x-rtp,media=video,encoding-name=VP8,payload=96"))

    pipeline.set_state(Gst.State.PLAYING)

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

    async for msg in ws:
        print(f"WS message: {msg.data}")
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

app = web.Application()
app.router.add_get('/', index)
app.router.add_get('/video_client.js', javascript)
app.router.add_get('/ws', websocket_handler)

web.run_app(app, port=4664)
