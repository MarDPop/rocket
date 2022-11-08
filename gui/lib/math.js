
function dot3(u,v) {
    return u[0]*v[0] + u[1]*v[1] + u[2]*v[2];
}

function add3(x,b){
    return [x[0]+b[0],x[1]+b[1],x[2]+b[2]];
}

function mult3(x,a){
    return [x[0]*a,x[1]*a,x[2]*a];
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

    ECEF = new Array(ENU.length);
    for(let i = 0; i < ENU.length;i++) {
        let x = start[0] + mult3(east,ENU[0]);
        let y = start[1] + mult3(north,ENU[1]);
        let z = start[2] + mult3(up,ENU[2]);
        ECEF[i] = [x,y,z];
    }

    return ECEF;
}