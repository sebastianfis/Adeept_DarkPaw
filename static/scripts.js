document.addEventListener('DOMContentLoaded', () => {
    const video = document.getElementById('video');
    let pc = new RTCPeerConnection();
    let dataChannel;
    const socket = new WebSocket(`ws://${window.location.host}/ws`);

    socket.onopen = () => console.log("✅ WS connected");

    socket.onmessage = async ({ data }) => {
        const msg = JSON.parse(data);

        if (msg.sdp && msg.sdp.type === "offer") {
            await pc.setRemoteDescription(new RTCSessionDescription(msg.sdp));
            const answer = await pc.createAnswer();
            await pc.setLocalDescription(answer);
            socket.send(JSON.stringify({ sdp: pc.localDescription }));
        } else if (msg.ice) {
            await pc.addIceCandidate(new RTCIceCandidate(msg.ice));
        }
    };

    pc.onicecandidate = ({ candidate }) => {
        if (candidate) socket.send(JSON.stringify({ ice: candidate }));
    };

    pc.ontrack = (event) => {
        console.log("📺 Track received", event);
        video.srcObject = event.streams[0];
        video.muted = true;
        video.play();
    };

    pc.ondatachannel = (event) => {
        console.log("📡 Data channel received");
        dataChannel = event.channel;

        dataChannel.onopen = () => {
            console.log("✅ Data channel open");
            setInterval(() => {
                if (dataChannel.readyState === "open") dataChannel.send("request_status");
            }, 200);
        };

        dataChannel.onmessage = (evt) => {
            try {
                const data = JSON.parse(evt.data);
                if (data.type === "status_update") {
                    document.getElementById("DistValue").innerHTML = data.Distance + " cm";
                    document.getElementById("RaspiCoreTemp").innerHTML = data.CPU_temp + " deg C";
                    document.getElementById("RaspiCPULoad").innerHTML = data.CPU_load + " %";
                    document.getElementById("RaspiRAMUsage").innerHTML = data.RAM_usage + " %";
                }
            } catch {}
        };
    };

    window.Btn_Click = (cmd) => {
        if (dataChannel?.readyState === "open") dataChannel.send(cmd);
    };

    window.addEventListener('beforeunload', () => {
        if (pc) pc.close();
    });
});
