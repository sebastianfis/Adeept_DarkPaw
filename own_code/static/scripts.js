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
//
//const velocity_value = document.querySelector("#velocity_value");
//const velocity_input = document.querySelector("#velocity_slider");
//velocity_value.textContent = velocity_input.value;
//velocity_input.addEventListener("velocity_input", (event) => {
//  velocity_value.textContent = event.target.value + " %";
//  fetch('/process_velocity_change/'+ event.target.value)
//        .then(request => request.text(event.target.value))
//});

//var slider = document.getElementById("velocity_slider");
//var output = document.getElementById("velocity_value");
//output.value = slider.value; // Display the default slider value
//
//// Update the current slider value (each time you drag the slider handle)
//slider.oninput = function() {
//  output.value = this.value;
//  fetch('/process_velocity_change/'+ this.value)
//        .then(request => request.text(this.value))
//}

document.getElementById("velocity_slider").oninput = function() {
   var val = document.getElementById("velocity_slider").value; //gets the oninput value
   document.getElementById('velocity_value').innerHTML = val; //displays this value to the html page
   fetch('/process_velocity_change/'+ val)
        .then(request => request.text(val))
};
