function Btn_Click(command) {
    fetch('/process_button_click/'+ command)
        .then(request => request.text(command))
}

const video_socket = io();
const info_socket = io();

        video_socket.on('frame', (jpg_as_text) => {
            const img = document.getElementById('camera_frame');
            img.src = 'data:image/jpeg;base64,' + jpg_as_text;
        });

