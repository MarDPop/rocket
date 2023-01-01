Cesium.Ion.defaultAccessToken = 'eyJhbGciOiJIUzI1NiIsInR5cCI6IkpXVCJ9.eyJqdGkiOiI1MWQ4Nzc2Mi01ZTFmLTQ5ZTQtYmI0Zi00MjZmYzk2YmJjNGYiLCJpZCI6MTExMTQsInNjb3BlcyI6WyJhc3IiLCJnYyJdLCJpYXQiOjE1NTgxMTk5Mjl9.OnV7lmXzp-fAUIEgsaIZs7pcOuTd8hPdLhyW-UyL7EA';
// Initialize the Cesium Viewer in the HTML element with the `cesiumContainer` ID.
const viewer = new Cesium.Viewer('cesiumContainer', {
    terrainProvider: Cesium.createWorldTerrain()
}); 

var rocketModel;

(async () => {
    try {
        rocketModel = await Cesium.IonResource.fromAssetId(1403984);     
    } catch (error) {
        console.log(error);
    }
})();

const scene = viewer.scene;

function plotData(data) {
    let seconds = data[0];
    let ecef = data[1];
    let quat = data[2];

    console.log(quat);
    
    const start = Cesium.JulianDate.fromIso8601("2023-03-09T13:10:00Z");
    const stop = Cesium.JulianDate.addSeconds(start, seconds[seconds.length-1], new Cesium.JulianDate());
    viewer.clock.startTime = start.clone();
    viewer.clock.stopTime = stop.clone();
    viewer.clock.currentTime = start.clone();
    viewer.timeline.zoomTo(start, stop);
    // Speed up the playback speed 50x.
    viewer.clock.multiplier = 1;
    // Start playing the scene.
    viewer.clock.shouldAnimate = true;
    const positionProperty = new Cesium.SampledPositionProperty();
    const orientationProperty = new Cesium.SampledProperty(Cesium.Quaternion); 

    let COGz = 1.1 - 0.6;
    for(let i = 0; i < seconds.length;i++) {
        var time = Cesium.JulianDate.addSeconds(start, seconds[i], new Cesium.JulianDate());
        var position = new Cesium.Cartesian3(ecef[i][0],ecef[i][1],ecef[i][2]);
        var orientation = new Cesium.Quaternion(-quat[i][1],-quat[i][2],-quat[i][3],quat[i][0]);
        var position_shifted = new Cesium.Cartesian3();
        var COG = new Cesium.Cartesian3(COGz*(quat[i][1]*quat[i][3] + quat[i][2]*quat[i][0]),COGz*(quat[i][2]*quat[i][3] - quat[i][1]*quat[i][0]),COGz*(1 - 2*(quat[i][1]*quat[i][1] + quat[i][2]*quat[i][2])));
        Cesium.Cartesian3.add(position,COG,position_shifted);
        positionProperty.addSample(time, position_shifted);
        orientationProperty.addSample(time, orientation);
    }       

    Cesium.EntityView.defaultOffset3D.z = 0.5;

    var rocket = viewer.entities.add({
        availability: new Cesium.TimeIntervalCollection([ new Cesium.TimeInterval({ start: start, stop: stop }) ]),
        position: positionProperty,
        model: {
            uri: rocketModel,
        },
        orientation: orientationProperty,    
        path: new Cesium.PathGraphics({ width: 2 })
    });

    viewer.trackedEntity = rocket;
}

document.getElementById('loadFile').addEventListener('click', () => {
    window.ipcRender.invoke('openTrajectoryFile').then((data) => {
        plotData(data);
    })
    
});

document.getElementById('nav-close-link').addEventListener('click', () => {
    let el = document.getElementById('input-container');
    el.style.width = 0;
    el.style.display = "none";
    el = document.getElementById('show-input');
    el.style.display = "block";
    document.getElementById('content-container').style.left = 0;
});

document.getElementById('show-input').addEventListener('click', () => {
    let el = document.getElementById('input-container');
    el.style.width = "400px";
    el.style.display = "block";
    el = document.getElementById('show-input');
    el.style.display = "none";
    document.getElementById('content-container').style.left = "400px";
});

document.querySelectorAll('#content-tabs li').forEach(function(tab) {
    tab.addEventListener('click', () => {
        document.querySelector('#content-window div.selected').className = "";
        document.querySelector('#content-tabs li.selected').className = "";

        let windowName = tab.getAttribute('data-window');

        document.getElementById(windowName).className = "selected";
        tab.className = "selected";
    });
});

function get_rocket_data() {
    // generate defaults
    var data = {
        "full" : {"mass" : 100 , "Ixx" : 19, "Izz" : 1.4, "COG" : -0.5},
        "empty" : {"mass" : 50, "Ixx" : 10, "Izz" : 0.7, "COG" : -0.5},
        "ground" : {"alt" : 0, "pressure" : 101325, "temperature" : 297, "lapse_rate" : 0, "heading" : 0, "pitch" : 0, "wind_file": null},
        "aero" : {"CD0" : 0.6, "CL_alpha" : 5, "CM_alpha" : -0.002, "CM_alpha_dot" : -0.08, "K" : 0.05, "area" : 0.07, "length" : 1.1, "stall_angle" : 0.2},
        "thruster" : [  
            {"pressure" : 105000, "thrust" : 3000, "mass_rate" : 3 },
            {"pressure" : 75000, "thrust" : 3000, "mass_rate" : 3 },
            {"pressure" : 10, "thrust" : 3000, "mass_rate" : 3 }
                    ],
        "NFINS" : 3,
        "finAero" : {"dSCL" : 0.1, "dSCM" : 0.001, "dSCD" : 0.001, "COP_z" : 1, "COP_radial" : 0.15},
        "finControl" : {"K1" : 0.1,"K2" : 0.1,"C2" : 0.2,"slew" : 0.5,"limit" : 0.1},
        "chute" : {"A0" : 0.1,"AF" : 1.5,"CD0" : 0.5, "CDF" : 1, "length" : 1,"deploy_time" : 5 }
    };

    document.getElementById("input_full_mass").value

    return data;
}

function load_nav() { 

    document.querySelectorAll('#form-nav a').forEach(element => {
        element.addEventListener('click', () =>{
            document.querySelectorAll('#form-nav a').forEach(el => {
                el.className = "";
            })
            document.querySelectorAll('#rocket_parameters table').forEach(el => {
                el.className = "";
            })
            
            element.className = "selected";
            let tablename = element.href.substring(element.href.indexOf('#')+1);
            document.getElementById(tablename).className = "selected";
        });
    });

    document.getElementById('runSim').addEventListener('click', () => {
        var data = get_rocket_data();
        window.ipcRender.invoke('runSim', data).then( (traj) => {
            if(traj == null) {
                return;
            }
            console.log(traj);
            plotData(traj);
        });
    })
}

fetch('./view/rocket_form.html').then((result) => {
    if (result.status != 200) { throw new Error("Bad Server Response"); }
    return result.text();
}).then((content) => {
    document.getElementById('rocket_form').innerHTML = content;
    load_nav();
}).catch((error) => { console.log(error); });

function load_analysis() {
    
}

fetch('./view/analysis_tab.html').then((result) => {
    if (result.status != 200) { throw new Error("Bad Server Response"); }
    return result.text();
}).then((content) => {
    document.getElementById('anaylsis-window').innerHTML = content;
    load_analysis();
}).catch((error) => { console.log(error); });

console.log("App loaded.");