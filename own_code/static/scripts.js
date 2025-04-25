let pc = new RTCPeerConnection();
let dataChannel;
let socket = new WebSocket('ws://' + window.location.host + '/ws');

socket.onmessage = async (event) => {
    const msg = JSON.parse(event.data);
    if (msg.answer) {
        await pc.setRemoteDescription(new RTCSessionDescription(msg.answer));
    } else if (msg.candidate) {
        await pc.addIceCandidate(new RTCIceCandidate(msg.candidate));
    }
};

pc.onicecandidate = ({ candidate }) => {
    if (candidate) {
        socket.send(JSON.stringify({ candidate }));
    }
};

dataChannel = pc.createDataChannel("control");

dataChannel.onopen = () => {
    console.log("Data channel open");
    callme();  // Start polling once data channel is open
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

(async () => {
    const offer = await pc.createOffer();
    await pc.setLocalDescription(offer);
    socket.send(JSON.stringify({ offer }));
})();

// ----- Command Functions -----

function Btn_Click(command) {
    if (dataChannel.readyState === "open") {
        dataChannel.send(command);
    } else {
        console.warn("Data channel not open");
    }
}

function Set_actuator() {
    let e = document.getElementById("ilazbj");
    let act_number = e.options[e.selectedIndex].text;
    let set_value = document.getElementById("i3r4u1").value;
    let command = 'setpwm_' + act_number + ':' + set_value;
    Btn_Click(command);
};

function Set_velocity() {
    let set_value = document.getElementById("velocity_slider").value;
    document.getElementById("velocity_value").innerHTML = "Set velocity: " + set_value + " %";
    Btn_Click('velocity_' + set_value);
}

function ChangeMode() {
    let set_mode = document.getElementById("mode_select").value;
    Btn_Click('mode_select:' + set_mode);
}

// ----- Periodic Status Request -----

function callme() {
    if (dataChannel.readyState === "open") {
        dataChannel.send("request_status");
    }
    setTimeout(callme, 200);  // Call again after 200ms
}
