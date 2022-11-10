
function dot3(u,v) {
    return u[0]*v[0] + u[1]*v[1] + u[2]*v[2];
}

function add3(x,b){
    return [x[0]+b[0],x[1]+b[1],x[2]+b[2]];
}

function mult3(x,a){
    return [x[0]*a,x[1]*a,x[2]*a];
}


function latLon2ECEF(lat,lon,alt) {
    let s = Math.sin(lat);
    let c = Math.sqrt(1-s*s);
    let N = 6378.1370/Math.sqrt(1 - 0.006694384442042289*s*s);
    let tmp = (N + alt)*c;
    let x = tmp*Math.cos(lon);
    let y = tmp*Math.sin(lon);
    let z = (0.9933056155579577*N + alt)*s;
    return [x,y,z];
}

function convertLocalENU2ECEF(ENU, lat, lon,elevation){
    let start = latLon2ECEF(lat,lon,elevation);

    const start_km = [start[0]*1000,start[1]*1000,start[2]*1000];

    let east = [-Math.sin(lon),Math.cos(lon),0];
    let north = [-Math.sin(lat)*Math.cos(lon), -Math.sin(lat)*Math.sin(lon), Math.cos(lat)];
    let up = [Math.cos(lat)*Math.cos(lon), Math.cos(lat)*Math.sin(lon), Math.sin(lat)];

    ECEF = new Array(ENU.length);
    for(let i = 0; i < ENU.length;i++) {
        ECEF[i] = add3(start_km,add3(mult3(east,ENU[i][0]),add3(mult3(north,ENU[i][1]),mult3(up,ENU[i][2]))));
    }

    return ECEF;
}

module.exports = {convertLocalENU2ECEF}