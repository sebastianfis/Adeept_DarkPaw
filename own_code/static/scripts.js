document.addEventListener('DOMContentLoaded', () => {
    if (!window.pc) {
        // Create the peer connection only once
        window.pc = new RTCPeerConnection();
    }
    let pc = window.pc;
    const socket = new WebSocket('ws://' + window.location.host + '/ws');
    const video = document.getElementById('video');

    socket.onopen = async () => {
        console.log("✅ WebSocket connected");

        // Create the offer and send it only after the WebSocket connection is open
        if (!pc.localDescription) {
            const offer = await pc.createOffer();
            await pc.setLocalDescription(offer);
            sendWebSocketMessage(JSON.stringify({ offer }));
            console.log("📤 Sending offer SDP");
        }
    };

    socket.onmessage = async ({ data }) => {
        console.log("📩 WS message from server:", data);
        const msg = JSON.parse(data);

        if (msg.sdp) {
            console.log("📜 Received SDP:", msg.sdp.type);
            await pc.setRemoteDescription(new RTCSessionDescription(msg.sdp));
            if (msg.sdp.type === "offer") {
                const answer = await pc.createAnswer();
                await pc.setLocalDescription(answer);
                sendWebSocketMessage(JSON.stringify({ sdp: pc.localDescription }));
                console.log("📤 Sending answer SDP");
            }
        } else if (msg.ice) {
            console.log("➕ Adding ICE candidate from server");
            await pc.addIceCandidate(new RTCIceCandidate(msg.ice));
        }
    };

    function sendWebSocketMessage(message) {
        if (socket.readyState === WebSocket.OPEN) {
            socket.send(message);
        } else {
            console.warn("WebSocket is not open. Retrying...");
            setTimeout(() => sendWebSocketMessage(message), 1000); // Retry after 1 second
        }
    }

    // Handle ICE candidates
    pc.onicecandidate = ({ candidate }) => {
        if (candidate) {
            console.log('🧊 Local ICE candidate:', candidate);
            sendWebSocketMessage(JSON.stringify({ ice: candidate }));
        }
    };

    // Handle media tracks
    pc.ontrack = (event) => {
        console.log("📺 Received track:", event);
        if (video && video instanceof HTMLVideoElement) {
            video.srcObject = event.streams[0];
            video.muted = true;
            video.play().catch(error => {
                console.error("Error playing video:", error);
            });
        } else {
            console.error("❌ Video element not found.");
        }
    };

    // ----- Command Functions -----
    window.Btn_Click = function(command) {
        if (dataChannel.readyState === "open") {
            dataChannel.send(command);
        } else {
            console.warn("Data channel not open");
        }
    }

    window.Set_actuator = function() {
        let e = document.getElementById("ilazbj");
        let act_number = e.options[e.selectedIndex].text;
        let set_value = document.getElementById("i3r4u1").value;
        let command = 'setpwm_' + act_number + ':' + set_value;
        Btn_Click(command);
    };

    window.Set_velocity = function() {
        let set_value = document.getElementById("velocity_slider").value;
        document.getElementById("velocity_value").innerHTML = "Set velocity: " + set_value + " %";
        Btn_Click('velocity_' + set_value);
    }

    window.ChangeMode = function() {
        let set_mode = document.getElementById("mode_select").value;
        Btn_Click('mode_select:' + set_mode);
    }

    // ----- Periodic Status Request -----

    function callme() {
        if (dataChannel.readyState === "open") {
            dataChannel.send("request_status");
        }
    }
});
