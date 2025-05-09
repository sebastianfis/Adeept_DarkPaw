document.addEventListener('DOMContentLoaded', () => {
    if (!window.pc) {
        // Create the peer connection only once
        window.pc = new RTCPeerConnection();
    }
    let pc = window.pc;
    let dataChannel;
    const socket = new WebSocket('ws://' + window.location.host + '/ws');
    const video = document.getElementById('video');

    socket.onopen = async () => {
        console.log("✅ WebSocket connected");

        // 🆕 Create a DataChannel only after connection is established (server will create offer)
        if (!dataChannel || dataChannel.readyState === "closed") {
            dataChannel = pc.createDataChannel("control");
            console.log("📡 Outgoing data channel created");

            dataChannel.onopen = () => {
                console.log("✅ Data channel open");
                setInterval(callme, 200); // Start periodic polling
            };

            dataChannel.onmessage = (event) => {
                try {
                    let data = JSON.parse(event.data);
                    if (data.type === "status_update") {
                        document.getElementById("DistValue").innerHTML = data.Distance + " cm";
                        document.getElementById("RaspiCoreTemp").innerHTML = data.CPU_temp + " deg C";
                        document.getElementById("RaspiCPULoad").innerHTML = data.CPU_load + " %";
                        document.getElementById("RaspiRAMUsage").innerHTML = data.RAM_usage + " %";
                    } else {
                        console.log("Received:", data);
                    }
                } catch (err) {
                    console.warn("Non-JSON or unknown message:", event.data);
                }
            };
        }
    };

    socket.onerror = (err) => {
        console.error("❌ WebSocket error:", err);
    };

    function sendWebSocketMessage(message) {
        if (socket.readyState === WebSocket.OPEN) {
            socket.send(message);
        } else {
            console.warn("WebSocket is not open. Retrying...");
            setTimeout(() => sendWebSocketMessage(message), 1000); // Retry after 1 second
        }
    }

    socket.onmessage = async ({ data }) => {
        console.log("📩 WS message from server:", data);
        const msg = JSON.parse(data);

        if (msg.sdp) {
            console.log("📜 Received SDP:", msg.sdp.type);

            // If we receive an offer from the server:
            if (msg.sdp.type === "offer") {
                console.log("📩 Received offer SDP");

                // Set the remote description (offer from server)
                await pc.setRemoteDescription(new RTCSessionDescription(msg.sdp));

                // Now create an answer to send back to the server
                const answer = await pc.createAnswer();
                await pc.setLocalDescription(answer);
                console.log("📤 Sending answer SDP");

                // Send the answer back to the server
                sendWebSocketMessage(JSON.stringify({ sdp: pc.localDescription }));
            }
        } else if (msg.ice) {
            console.log("➕ Adding ICE candidate from server");
            await pc.addIceCandidate(new RTCIceCandidate(msg.ice));
        }
    };

    pc.onicecandidate = ({ candidate }) => {
        if (candidate) {
            console.log('🧊 Local ICE candidate:', candidate);
            sendWebSocketMessage(JSON.stringify({ ice: candidate }));
        }
    };

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

    pc.ondatachannel = (event) => {
        console.log("📡 Data channel received on client");
        dataChannel = event.channel;
        console.log("📡 Data channel received: ", dataChannel);

        dataChannel.onopen = () => {
            console.log("✅ Data channel open on client");
            setInterval(callme, 200); // Start periodic polling
        };

        dataChannel.onmessage = (event) => {
            try {
                let data = JSON.parse(event.data);
                if (data.type === "status_update") {
                    document.getElementById("DistValue").innerHTML = data.Distance + " cm";
                    document.getElementById("RaspiCoreTemp").innerHTML = data.CPU_temp + " deg C";
                    document.getElementById("RaspiCPULoad").innerHTML = data.CPU_load + " %";
                    document.getElementById("RaspiRAMUsage").innerHTML = data.RAM_usage + " %";
                } else {
                    console.log("Received:", data);
                }
            } catch (err) {
                console.warn("Non-JSON or unknown message:", event.data);
            }
        };
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

    window.addEventListener('beforeunload', () => {
        if (window.pc) {
            window.pc.close();
            window.pc = null; // Reset the peer connection
            console.log("✅ WebRTC peer connection closed");
        }
    });
});
