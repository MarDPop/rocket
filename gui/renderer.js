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

document.getElementById('loadFile').addEventListener('click', () => {
    window.ipcRender.invoke('openTrajectoryFile').then((data) => {
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
    })
    
});

function loadPage(){ 

    document.querySelectorAll('form a').forEach(element => {
        element.addEventListener('click', () =>{
            document.querySelectorAll('form a').forEach(el => {
                el.className = "";
            })
            document.querySelectorAll('form table').forEach(el => {
                el.className = "";
            })
            
            element.className = "selected";
            let tablename = element.href.substring(element.href.indexOf('#')+1);
            console.log(tablename);
            document.getElementById(tablename).className = "selected";
        });
    });

    document.getElementById('runSim').addEventListener('click', () => {
        var data = {};
        window.ipcRender.invoke('runSim',data).then(() => {
    
        });
    })
}

fetch('./view/rocket_form.html').then((result) => {
    if (result.status != 200) { throw new Error("Bad Server Response"); }
    return result.text();
}).then((content) => {
    document.getElementById('rocket_form').innerHTML = content;
    loadPage();
}).catch((error) => { console.log(error); });

console.log("App loaded.");