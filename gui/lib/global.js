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
        let alt = parseFloat(row2[2]);
        
        const start = math.latLon2ECEF(lat,lon,alt);
        const CS = math.getENUinECEF(lat,lon);

        console.log(CS);

        let A = [[1,0,0],[0,1,0],[0,0,1]];
        let B = new Array(3);
        this.times = new Array(n);
        this.position_ECEF = new Array(n);
        this.orientation_ECEF = new Array(n);
        for(let i = 0; i < n; i++){
            let row = lines[i+1].trim().split(/\s+/);
            this.times[i] = parseFloat(row[0]);
            let pos = [parseFloat(row[1]),parseFloat(row[2]),parseFloat(row[3])];
            this.position_ECEF[i] = math.add3(start,math.multmatT3(CS,pos));
            let j = 4;
            for(let r = 0; r < 3; r++){
                for(let c = 0; c < 3; c++){
                    A[r][c] = parseFloat(row[j]);
                    j++;
                }
            }

            // Y up
            B[0] = math.multmatT3(CS,A[2]);
            B[1] = math.multmatT3(CS,A[0]);
            B[2] = math.multmatT3(CS,A[1]);

            this.orientation_ECEF[i] = math.mat2quat(B);
        }
        //console.log(this.orientation_ECEF);
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
    let tmp = data["empty"];
    let empty_mass = tmp["mass"] + " " + tmp["Ixx"] + " " + tmp["Izz"] + " " + tmp["COG"] + "\n";
    tmp = data["full"];
    let full_mass = tmp["mass"] + " " + tmp["Ixx"] + " " + tmp["Izz"] + " " + tmp["COG"] + "\n";
    tmp = data["ground"];
    let ground = tmp["alt"] + " " + tmp["pressure"] + " " + tmp["temperature"] + " " + tmp["lapse_rate"] + "\n";
    let windFile = tmp["wind_file"] == null ? "" : tmp["wind_file"] ;
    let direction = tmp["heading"] + " " + tmp["pitch"] + " " + windFile  + "\n";
    tmp = data["aero"];
    let aero = tmp["CD0"] + " " + tmp["CL_alpha"] + " " + tmp["CM_alpha"] + " " + tmp["CM_alpha_dot"] + " " + tmp["K"] + " " + tmp["area"] + " " + tmp["length"] + " " + tmp["stall_angle"] + "\n";
    tmp = data["thruster"];
    let thruster = "NPTS " + tmp.length + "\n"
    for(let i = 0; i < tmp.length;i++){
        thruster += tmp[i]["pressure"] + " " + tmp[i]["thrust"] + " " + tmp[i]["mass_rate"] + "\n";
    }


    tmp = data["control"]["fin_aero"];
    let fins = data["control"]["NFINS"] + " " + tmp["dSCL"] + " " + tmp["dSCM"] + " " + tmp["dSCD"] + " " + tmp["COP_z"] + " " + tmp["COP_radial"] + "\n";
    tmp = data["control"]["fin_control"];
    let control = tmp["K1"] + " " + tmp["K2"] + " " + tmp["C2"] + " " + tmp["slew"] + " " + tmp["limit"] + "\n";
    tmp = data["control"]["chute"];
    let chute = tmp["A0"] + " " + tmp["AF"] + " " + tmp["CD0"] + " " + tmp["CDF"] + " " + tmp["length"] + " " + tmp["deploy_time"] + "\n";

    let output = "# mass, Ixx, Izz, COG (Empty)\n" +
    empty_mass +
    "# mass, Ixx, Izz, COG (Full) \n" + 
    full_mass +
    "# Alt, Pres, Temp, Lapse Rate \n" + 
    ground +
    "# Heading, Pitch, Wind File  \n" + 
    direction +
    "# CD0, CL_a, CM_a, CM_a_dot, K, area, length, stall  \n" + 
    aero +
    "# thruster points  \n" + 
    thruster +
    "# NFINS, dSCL, dSCM, dSCD, COP_z, COP_radial  \n" + 
    fins +
    "# K1,K2,C2,slew,limit  \n" + 
    control +
    "# A0,AF,CD0,CDF,length,deploy time  \n" + 
    chute;

    fs.writeFile('data/rocket.srocket', output, (err) => {
        if (err) throw err;
        else{
           console.log("The file is updated with the given data")
        }
    })
}

module.exports = {DATA,writeRocketFile};