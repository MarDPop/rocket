const dialog = require('electron').dialog
const vector = require('math')

function openMissionFile() {
    // construct the select file dialog 
    dialog.showOpenDialog({
        properties: ['openFile']
    })
    .then(function(fileObj) {
        if (!fileObj.canceled) {
            mainWindow.webContents.send('FILE_OPEN', fileObj.filePaths);
            functions.openMissionFile(fileObj.filePaths);
        }
    })
    .catch(function(err) {
        console.error(err)  
    })
}

function latLon2ECEF(lat,lon) {
    var s = Math.sin(lat);
    var c = Math.sqrt(1-s*s);
    var N = 6378.1370/Math.sqrt(1 - 0.006694384442042289*s*s);
    var x = N*c*Math.cos(lon);
    var y = N*c*Math.sin(lon);
    var z = 0.9933056155579577*N*s;
    return [x,y,z];
}

function convertLocalENU2ECEF(ENU, lat, lon){
    var start = latLon2ECEF(lat,lon);

    var east = [-Math.sin(lon),Math.cos(lon),0];
    var north = [-Math.sin(lat)*Math.cos(lon), -Math.sin(lat)*Math.sin(lon), Math.cos(lat)];
    var up = [Math.cos(lat)*Math.cos(lon), Math.cos(lat)*Math.sin(lon), Math.sin(lat)];

    
}


module.exports = {openMissionFile};