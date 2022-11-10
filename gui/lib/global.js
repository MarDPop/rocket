var fs = require('fs');
var math = require('./math');

class Trajectory {
    times = []
    position_ENU = []
    position_ECEF = []

    constructor(){
        this.load = this.load.bind(this);
    }

    load(fn) {
        var data;
        try {
            data = fs.readFileSync(fn,  {encoding:'utf8', flag:'r'});
        } catch (e) {
            return;
        }
        var t = this;
        let lines = data.split(/\r?\n/);
        let n = lines.length - 1;
        this.times = new Array(n);
        this.position_ENU = new Array(n);
        for(let i = 0; i < n; i++){
            let row = lines[i+1].split(/\s+/);
            t.times[i] = Number.parseFloat(row[1]);
            t.position_ENU[i] = [parseFloat(row[2]),parseFloat(row[3]),parseFloat(row[4])];
        }
        let row2 = lines[0].split(/\s+/);
        let lat = parseFloat(row2[0])*0.0174532925;
        let lon = parseFloat(row2[1])*0.0174532925;
        let alt = parseFloat(row2[2])*0.001;
        this.position_ECEF = math.convertLocalENU2ECEF(this.position_ENU,lat,lon,alt);
        console.log("Trajectory loaded.");
    }
}

class MissionData {

    trajectory = new Trajectory();

    constructor() {
    }

    load(data) {
        
    }
}

const DATA = new MissionData();

function writeRocketFile(data){

}

module.exports = {DATA,writeRocketFile};