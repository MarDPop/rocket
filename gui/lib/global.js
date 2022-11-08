var mission = require('./missionData');

var DATA = new mission.MissionData();

function openMissionFile(file) {
    console.log("Opening Mission File: " + fn);
}

function openTrajectoryFile(file){
    lines = file.split(/\r?\n/);
    for(let i = 0; i < lines.length;i++){
        console.log(lines[i]);
    }
}


module.exports = {DATA,openMissionFile, openTrajectoryFile};