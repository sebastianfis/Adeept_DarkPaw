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

    pipeline = Gst.parse_launch(
        'videotestsrc is-live=true ! videoconvert ! vp8enc deadline=1 ! rtpvp8pay ! webrtcbin name=sendrecv'
    )
    webrtc = pipeline.get_by_name('sendrecv')

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

        text = json.dumps({'sdp': {
            'type': 'offer',
            'sdp': offer.sdp.as_text()
        }})
        asyncio.run_coroutine_threadsafe(ws.send_str(ice), loop)

    def on_ice_candidate(_, mlineindex, candidate):
        ice = json.dumps({'ice': {
            'candidate': candidate,
            'sdpMLineIndex': mlineindex,
        }})
        asyncio.run_coroutine_threadsafe(ws.send_str(ice), loop)

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

app = web.Application()
app.router.add_get('/', index)
app.router.add_get('/video_client.js', javascript)
app.router.add_get('/ws', websocket_handler)

web.run_app(app, port=4664)
