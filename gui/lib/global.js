var fs = require('fs');
var math = require('./math');

class Trajectory {
    times = []
    position_ECEF = []
    orientation_ECEF = []

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
        let lines = data.trim().split(/\r?\n/);

        let n = lines.length - 1;
        if(n < 1){
            return;
        }

        let row2 = lines[0].split(/\s+/);
        let lat = parseFloat(row2[0])*0.0174532925;
        let lon = parseFloat(row2[1])*0.0174532925;
        let alt = parseFloat(row2[2])*0.001;
        
        let start = math.latLon2ECEF(lat,lon,alt);
        let CS = math.getENUinECEF(lat,lon);
        const start_meters = [start[0]*1000,start[1]*1000,start[2]*1000];

        let A = [[1,0,0],[0,1,0],[0,0,1]];
        let B = new Array(3);
        this.times = new Array(n);
        this.position_ENU = new Array(n);
        this.orientation_ECEF = new Array(n);
        for(let i = 0; i < n; i++){
            let row = lines[i+1].split(/\s+/);
            this.times[i] = parseFloat(row[0]);
            let pos = [parseFloat(row[1]),parseFloat(row[2]),parseFloat(row[3])];
            this.position_ECEF[i] = math.add3(start_meters,math.multmatT3(CS,pos));
            let j = 4;
            for(let r = 0; r < 3; r++){
                for(let c = 0; c < 3; c++){
                    A[r][c] = parseFloat(row[j]);
                    j++;
                }
            }
            
            for(let r = 0; r < 3; r++){
                B[r] = math.multmatT3(CS,A[r]);
            }

            this.orientation_ECEF[i] = [1,0,0,0];// math.mat2quat(B);
        }
        console.log(this.orientation_ECEF);
        
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