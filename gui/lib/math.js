
function dot3(u,v) {
    return u[0]*v[0] + u[1]*v[1] + u[2]*v[2];
}

function add3(x,b){
    return [x[0]+b[0],x[1]+b[1],x[2]+b[2]];
}

function mult3(x,a){
    return [x[0]*a,x[1]*a,x[2]*a];
}

function multmat3(A,x){
    return [dot3(A[0],x),dot3(A[1],x),dot3(A[2],x)];
}

function multmatT3(A,x){
    return add3(mult3(A[0],x[0]),add3(mult3(A[1],x[1]),mult3(A[2],x[2])));
}

function det(a){
    let x=a[0][0]*((a[1][1]*a[2][2])-(a[2][1]*a[1][2]));
    let y=-a[0][1]*((a[1][0]*a[2][2])-(a[2][0]*a[1][2]));
    let z=a[0][2]*((a[1][0]*a[2][1])-(a[1][1]*a[2][0]));

    return x+y+z;
}

function mat2quat(a){
    let trace = a[0][0] + a[1][1] + a[2][2];
    var q = [0,0,0,0];
    if( trace > 0 ) {
        let s = 0.5 / Math.sqrt(trace + 1.0);
        q[0] = 0.25 / s;
        q[1] = ( a[2][1] - a[1][2] ) * s;
        q[2] = ( a[0][2] - a[2][0] ) * s;
        q[3] = ( a[1][0] - a[0][1] ) * s;
    } else {
        if ( a[0][0] > a[1][1] && a[0][0] > a[2][2] ) {
            let s = 2.0 * Math.sqrt( 1.0 + a[0][0] - a[1][1] - a[2][2]);
            q[0] = (a[2][1] - a[1][2] ) / s;
            q[1] = 0.25 * s;
            q[2] = (a[0][1] + a[1][0] ) / s;
            q[3] = (a[0][2] + a[2][0] ) / s;
        } else if (a[1][1] > a[2][2]) {
            let s = 2.0 * Math.sqrt( 1.0 + a[1][1] - a[0][0] - a[2][2]);
            q[0] = (a[0][2] - a[2][0] ) / s;
            q[1] = (a[0][1] + a[1][0] ) / s;
            q[2] = 0.25 * s;
            q[3] = (a[1][2] + a[2][1] ) / s;
        } else {
            let s = 2.0 * Math.sqrt( 1.0 + a[2][2] - a[0][0] - a[1][1] );
            q[0] = (a[1][0] - a[0][1] ) / s;
            q[1] = (a[0][2] + a[2][0] ) / s;
            q[2] = (a[1][2] + a[2][1] ) / s;
            q[3] = 0.25 * s;
        }
    }
    // although not strictly necessary, this seems to trip up cesium if not close enough
    let sq = q[0]*q[0] + q[1]*q[1] + q[2]*q[2] + q[3]*q[3];
    let n = 1.0/Math.sqrt(sq);
    q[0] *= n;
    q[1] *= n;
    q[2] *= n;
    q[3] *= n;
    return q;
}

function latLon2ECEF(lat,lon,alt) {
    let s = Math.sin(lat);
    let c = Math.sqrt(1-s*s);
    let N = 6378137.0/Math.sqrt(1 - 0.006694384442042289*s*s);
    let tmp = (N + alt)*c;
    let x = tmp*Math.cos(lon);
    let y = tmp*Math.sin(lon);
    let z = (0.9933056155579577*N + alt)*s;
    return [x,y,z];
}

function getENUinECEF(lat, lon) {
    let sLon = Math.sin(lon);
    let cLon = Math.cos(lon);
    let sLat = Math.sin(lat);
    let cLat = Math.sqrt(1-sLat*sLat);
    let east = [-sLon,cLon,0];
    let north = [-sLat*cLon, -sLat*sLon, cLat];
    let up = [cLat*cLon, cLat*sLon, sLat];
    return [east,north,up];
}

module.exports = {getENUinECEF,latLon2ECEF,dot3,add3,mult3,multmat3,multmatT3,mat2quat,det}