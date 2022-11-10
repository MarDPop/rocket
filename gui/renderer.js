

Cesium.Ion.defaultAccessToken = 'eyJhbGciOiJIUzI1NiIsInR5cCI6IkpXVCJ9.eyJqdGkiOiI1MWQ4Nzc2Mi01ZTFmLTQ5ZTQtYmI0Zi00MjZmYzk2YmJjNGYiLCJpZCI6MTExMTQsInNjb3BlcyI6WyJhc3IiLCJnYyJdLCJpYXQiOjE1NTgxMTk5Mjl9.OnV7lmXzp-fAUIEgsaIZs7pcOuTd8hPdLhyW-UyL7EA';
// Initialize the Cesium Viewer in the HTML element with the `cesiumContainer` ID.
const viewer = new Cesium.Viewer('cesiumContainer', {
    terrainProvider: Cesium.createWorldTerrain()
}); 

var rocketModel;

(async () => {
    try {
        rocketModel = await Cesium.IonResource.fromAssetId(1395959);     
    } catch (error) {
        console.log(error);
    }
})();

const scene = viewer.scene;

document.getElementById('runSim').addEventListener('click', () => {
    var data = {};
    window.ipcRender.invoke('runSim',data).then(() => {

    });
})

document.getElementById('loadFile').addEventListener('click', () => {
    window.ipcRender.invoke('openTrajectoryFile').then((data) => {
        let seconds = data[0];
        let ecef = data[1];
        
        const timeStepInSeconds = 0.25;
        const totalSeconds = timeStepInSeconds * (seconds.length - 1);
        const start = Cesium.JulianDate.fromIso8601("2020-03-09T23:10:00Z");
        const stop = Cesium.JulianDate.addSeconds(start, totalSeconds, new Cesium.JulianDate());
        viewer.clock.startTime = start.clone();
        viewer.clock.stopTime = stop.clone();
        viewer.clock.currentTime = start.clone();
        viewer.timeline.zoomTo(start, stop);
        // Speed up the playback speed 50x.
        viewer.clock.multiplier = 1;
        // Start playing the scene.
        viewer.clock.shouldAnimate = true;
        const positionProperty = new Cesium.SampledPositionProperty();

        let pos = []
        for(let i = 0; i < ecef.length;i++){
            var position = new Cesium.Cartesian3(ecef[i][0],ecef[i][1],ecef[i][2]);
            pos.push(position);
            var time = Cesium.JulianDate.addSeconds(start, i * timeStepInSeconds, new Cesium.JulianDate());
            positionProperty.addSample(time, position);
        }       

        var greenLine = viewer.entities.add({
            polyline : {
                positions : pos,
                material : Cesium.Color.RED
            }
        });

        var rocket = viewer.entities.add({
            availability: new Cesium.TimeIntervalCollection([ new Cesium.TimeInterval({ start: start, stop: stop }) ]),
            position: positionProperty,
            model: {
                uri: rocketModel,
            },
            orientation: new Cesium.VelocityOrientationProperty(positionProperty),    
            path: new Cesium.PathGraphics({ width: 3 })
        });

        viewer.trackedEntity = rocket;
    })
    
})

console.log("App loaded.");