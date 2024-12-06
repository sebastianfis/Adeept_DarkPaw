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
//
//let i = document.querySelector('velocity_slider'),
//    o = document.querySelector('velocity_value');
//
//o.innerHTML = i.value;
//
//// use 'change' instead to see the difference in response
//i.addEventListener('input', function () {
//  o.innerHTML = i.value;
//}, false);

const slider = document.getElementById("velocity_slider");
var a = 100; //variable to be controlled

const disp = document.getElementById("id01");
disp.innerHTML = "Set velocity: " + a;

//function is called when slider value changes
slider.addEventListener("change", function() {
  a = slider.value;
  disp.innerHTML = "Set velocity: " + a;
  fetch('/process_velocity_change/'+ command)
        .then(request => request.text(command))
}, true);
//
//const slider = document.querySelector("#range4")
//const sliderValue = document.querySelector(".value4")
//
//sliderEl4.addEventListener("input", (event) => {
//  const tempSliderValue = event.target.value;
//  sliderValue4.textContent = tempSliderValue;
//
//  const progress = (tempSliderValue / sliderEl4.max) * 100;
//})
//
//
//document.getElementById("velocity_slider").oninput = function() {
//   var val = document.getElementById("velocity_slider").value; //gets the oninput value
//   document.getElementById('velocity_value').innerHTML = val; //displays this value to the html page
//
//};
