const pc = new RTCPeerConnection();
const ws = new WebSocket('ws://' + window.location.host + '/ws');
const video = document.getElementById('video');

pc.ontrack = (event) => {
  video.srcObject = event.streams[0];
};

pc.onicecandidate = ({ candidate }) => {
  if (candidate) {
    ws.send(JSON.stringify({ ice: candidate }));
  }
};

ws.onmessage = async ({ data }) => {
  const msg = JSON.parse(data);

  if (msg.sdp) {
    await pc.setRemoteDescription(new RTCSessionDescription(msg.sdp));
    const answer = await pc.createAnswer();
    await pc.setLocalDescription(answer);
    ws.send(JSON.stringify({ sdp: pc.localDescription }));
  } else if (msg.ice) {
    await pc.addIceCandidate(new RTCIceCandidate(msg.ice));
  }
};