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
    document.getElementById("velocity_value").innerHTML = "Set velocity: " + set_value + " %";
    fetch('/process_velocity_change/'+ set_value)
         .then(request => request.text(set_value))
}

function ChangeMode() {
    var set_mode = document.getElementById("mode_select").value;
    fetch('/process_mode_change/'+ set_mode)
         .then(request => request.text(set_mode))
}

function callme(){
//This promise will resolve when the network call succeeds
//Feel free to make a REST fetch using promises and assign it to networkPromise
var networkPromise = fetch('/read_data')
  .then(response => response.json())
  .then(data => {
    document.getElementById("DistValue").innerHTML = data['Distance'] + " cm"
    document.getElementById("RaspiCoreTemp").innerHTML = data['CPU_temp'] + " deg C"
    document.getElementById("RaspiCPULoad").innerHTML = data['CPU_load'] + " %"
    document.getElementById("RaspiRAMUsage").innerHTML = data['RAM_usage'] + " %"
  });


//This promise will resolve when 200 ms seconds have passed
var timeOutPromise = new Promise(function(resolve, reject) {
  // 200 millisecond delay
  setTimeout(resolve, 200, 'Timeout Done');
});

Promise.all(
[networkPromise, timeOutPromise]).then(function(values) {
  callme();
});
}

window.addEventListener("DOMContentLoaded", (event) => {
    callme();
});