function Btn_Click(command) {
    fetch('/process_button_click/'+ command)
        .then(request => request.text(command))
}

function Set_actuator() {
    var e = document.getElementById("ilazbj");
    var act_number = e.options[e.selectedIndex].text;
    var set_value = document.getElementById("i3r4u1").value;
    command = 'setpwm_' + act_number + ':' + set_value;
    Btn_Click(command);
}


const video_socket = io();
const info_socket = io();

        video_socket.on('frame', (jpg_as_text) => {
            const img = document.getElementById('camera_frame');
            img.src = 'data:image/jpeg;base64,' + jpg_as_text;
        });

