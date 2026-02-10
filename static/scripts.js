document.addEventListener('DOMContentLoaded', () => {
    // Create PeerConnection only once
    if (!window.pc) window.pc = new RTCPeerConnection();
    let pc = window.pc;
    let dataChannel;
    const video = document.getElementById('video');

    // WebSocket to server
    const socket = new WebSocket('ws://' + window.location.host + '/ws');

    socket.onopen = () => {
        console.log("✅ WebSocket connected");
    };

    socket.onerror = (err) => {
        console.error("❌ WebSocket error:", err);
    };

    function sendWebSocketMessage(msg) {
        if (socket.readyState === WebSocket.OPEN) {
            socket.send(msg);
        } else {
            console.warn("WebSocket not open. Retrying...");
            setTimeout(() => sendWebSocketMessage(msg), 1000);
        }
    }

    socket.onmessage = async ({ data }) => {
        const msg = JSON.parse(data);

        if (msg.sdp) {
            console.log(`📜 Received SDP: ${msg.sdp.type}`);
            if (msg.sdp.type === "offer") {
                // Server created offer
                await pc.setRemoteDescription(new RTCSessionDescription(msg.sdp));

                // Create answer
                const answer = await pc.createAnswer();
                await pc.setLocalDescription(answer);
                console.log("📤 Sending SDP answer");

                sendWebSocketMessage(JSON.stringify({ sdp: pc.localDescription }));
            }
        } else if (msg.ice) {
            console.log("➕ Adding ICE candidate");
            await pc.addIceCandidate(new RTCIceCandidate(msg.ice));
        }
    };

    // Send local ICE candidates to server
    pc.onicecandidate = ({ candidate }) => {
        if (candidate) {
            sendWebSocketMessage(JSON.stringify({ ice: candidate }));
        }
    };

    // Attach remote video track
    pc.ontrack = (event) => {
        console.log("📺 Received remote track");
        if (video && event.streams && event.streams[0]) {
            video.srcObject = event.streams[0];
            video.muted = true;
            video.play().catch(e => console.error("❌ Video play error:", e));
        }
    };

    // Receive data channel from server
    pc.ondatachannel = (event) => {
        dataChannel = event.channel;
        console.log("📡 Data channel received");

        dataChannel.onopen = () => {
            console.log("✅ Data channel open");
            setInterval(requestStatus, 200); // periodic status request
        };

        dataChannel.onmessage = (event) => {
            try {
                const data = JSON.parse(event.data);
                if (data.type === "status_update") {
                    document.getElementById("DistValue").innerText = data.Distance + " cm";
                    document.getElementById("RaspiCoreTemp").innerText = data.CPU_temp + " °C";
                    document.getElementById("RaspiCPULoad").innerText = data.CPU_load + " %";
                    document.getElementById("RaspiRAMUsage").innerText = data.RAM_usage + " %";
                } else {
                    console.log("Received:", data);
                }
            } catch (err) {
                console.warn("Non-JSON message:", event.data);
            }
        };
    };

    // Periodic status request
    function requestStatus() {
        if (dataChannel && dataChannel.readyState === "open") {
            dataChannel.send("request_status");
        }
    }

    // Send commands
    function sendCommand(command) {
        if (dataChannel && dataChannel.readyState === "open") {
            dataChannel.send(command);
        } else {
            console.warn("Data channel not open");
        }
    }

    // ----- Command buttons -----
    window.Btn_Click = sendCommand;

    window.Set_actuator = function() {
        let e = document.getElementById("ilazbj");
        let act_number = e.options[e.selectedIndex].text;
        let set_value = document.getElementById("i3r4u1").value;
        Btn_Click('setpwm_' + act_number + ':' + set_value);
    };

    window.Set_velocity = function() {
        let set_value = document.getElementById("velocity_slider").value;
        document.getElementById("velocity_value").innerText = "Set velocity: " + set_value + " %";
        Btn_Click('velocity_' + set_value);
    };

    window.ChangeMode = function() {
        let set_mode = document.getElementById("mode_select").value;
        Btn_Click('mode_select:' + set_mode);
    };

    // Close peer connection on page unload
    window.addEventListener('beforeunload', () => {
        if (window.pc) {
            window.pc.close();
            window.pc = null;
            console.log("✅ WebRTC peer connection closed");
        }
    });
});
