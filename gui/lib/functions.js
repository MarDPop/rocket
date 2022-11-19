console.log("App loading...");


function addThrusterPoint(){
    let n = document.querySelectorAll("#thruster tr").length - 1;
    let row = '<tr>\n'+
        '<td><input type="text" id="input_PA_'+ n +'" name="input_PA_'+ n +'" /></td>\n'+
        '<td><input type="text" id="input_THRUST_'+ n +'" name="input_THRUST_'+ n +'" /></td>\n'+
        '<td><input type="text" id="input_MDOT_'+ n +'" name="input_MDOT_'+ n +'" /></td></tr>\n';
    document.getElementById("thruster").innerHTML += row;
}

function removeThrusterPoint() {
    let n = document.querySelectorAll("#thruster tr").length - 1;
    let el = document.querySelectorAll("#thruster tr")[n];
    el.parentNode.removeChild(el);
}