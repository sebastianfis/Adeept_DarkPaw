function Btn_Click(command) {
    fetch('/process_button_click/'+ command)
        .then(request => request.text(command));
};

function Set_actuator() {
    var e = document.getElementById("ilazbj");
    var act_number = e.options[e.selectedIndex].text;
    var set_value = document.getElementById("i3r4u1").value;
    command = 'setpwm_' + act_number + ':' + set_value;
    Btn_Click(command);
};

function Set_velocity() {
    var set_value = document.getElementById("velocity_slider").value;
    document.getElementById("velocity_value").innerHTML = "Set velocity: " + set_value;
    fetch('/process_velocity_change/'+ set_value)
         .then(request => request.text(set_value))
}
//
//window.addEventListener("DOMContentLoaded", (event) => {
//    var slider = document.getElementById("velocity_slider");
//    var a = 100; //variable to be controlled
//    var disp = document.getElementById("velocity_value");
//    disp.innerHTML = "Set velocity: " + a;
//
//    //function is called when slider value changes
//    slider.addEventListener("input", function() {
//    a = slider.value;
//    disp.innerHTML = "Set velocity: " + a;
//    fetch('/process_velocity_change/'+ command)
//         .then(request => request.text(command))
//});
//});
