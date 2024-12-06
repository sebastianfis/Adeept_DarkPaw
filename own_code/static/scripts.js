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

var slider = document.getElementById("velocity_slider");

var a = slider.value; //variable to be controlled
var dispDiv = document.getElementById("velocity_value");
dispDiv.innerHTML = "Set movement velocity: " + a;

//function is called when slider value changes
slider.addEventListener("change", function() {
  a = slider.value;
  dispDiv.innerHTML = "Set movement velocity: " + a;
  fetch('/process_velocity_change/'+ val)
        .then(request => request.text(val))
})
//
//
//document.getElementById("velocity_slider").oninput = function() {
//   var val = document.getElementById("velocity_slider").value; //gets the oninput value
//   document.getElementById('velocity_value').innerHTML = val; //displays this value to the html page
//
//};
