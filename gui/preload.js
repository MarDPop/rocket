const child_process = require("child_process");

function runExecutable(executablePath, args) {
    var child = child_process.execFile;
    var command = executablePath + " " + args;
    child(command, function(err, data) {
        if(err){
            console.error(err);
            return;
        }
        console.log(data.toString());
    });
};

window.addEventListener('DOMContentLoaded', () => {
    // do stuff when window loads
    var btn = document.getElementById("runSim");
    var btn = document.getElementById("rocketFile");
    btn.addEventListener('click', () => {runExecutable("bin\\rocket.exe","test.srocket");});
    console.log("loaded DOM.");
});
