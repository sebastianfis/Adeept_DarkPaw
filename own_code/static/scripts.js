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

var slider = document.getElementById("Velocity");
var output = document.getElementById("velocity_value");
output.innerHTML = slider.value; // Display the default slider value

// Update the current slider value (each time you drag the slider handle)
slider.oninput = function() {
  output.innerHTML = this.value;
  fetch('/process_velocity_change/'+ slider.value)
        .then(request => request.text(slider.value))
}