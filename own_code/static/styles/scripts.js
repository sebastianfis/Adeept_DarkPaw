function Btn_Click(command) {
    fetch('/process_button_click/'+ command)
        .then(request => request.text(command))
}

function Set_actuator() {
    var e = document.getElementById("ilazbj");
    var act_number = e.options[e.selectedIndex].text;
    var set_value = document.getElementById("i3r4u1").value;
    command = 'setpwm_' + act_number + ':' + set_value
    fetch('/process_button_click/'+ command)
        .then(request => request.text(command))
}

const video_socket = io();
const info_socket = io();

video_socket.on('frame', (jpg_as_text) => {
    const img = document.getElementById('camera_frame');
    img.src = 'data:image/jpeg;base64,' + jpg_as_text;
});

video_socket.addEventListener('message', ev => {
msg = JSON.parse(ev.data);
document.getElementById('clock').innerHTML = msg.text;
});
//FIXME: Significant lags in image stream, and commands receiving!
//
//const FPS = 30;
//document.addEventListener("DOMContentLoaded", function(event) {
//    const video_socket = io.connect(`ws://${document.domain}:${location.port}/camera-feed`);
//    video_socket.on('new-frame', message => {
//        document.getElementById('camera_frame').setAttribute(
//            'src', `data:image/jpeg;base64,${message.base64}`
//        );
//    });
//    window.setInterval(() => {
//        video_socket.emit('request-frame', {});
//    }, 1000/FPS);
//
//});

