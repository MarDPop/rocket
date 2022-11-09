
Cesium.Ion.defaultAccessToken = 'eyJhbGciOiJIUzI1NiIsInR5cCI6IkpXVCJ9.eyJqdGkiOiI1MWQ4Nzc2Mi01ZTFmLTQ5ZTQtYmI0Zi00MjZmYzk2YmJjNGYiLCJpZCI6MTExMTQsInNjb3BlcyI6WyJhc3IiLCJnYyJdLCJpYXQiOjE1NTgxMTk5Mjl9.OnV7lmXzp-fAUIEgsaIZs7pcOuTd8hPdLhyW-UyL7EA';
// Initialize the Cesium Viewer in the HTML element with the `cesiumContainer` ID.
const viewer = new Cesium.Viewer('cesiumContainer', {
    terrainProvider: Cesium.createWorldTerrain()
}); 

const scene = viewer.scene;

document.getElementById('runSim').addEventListener('click', () => {
    var data = {};
    window.ipcRender.invoke('runSim',data).then(() => {

    });
})

document.getElementById('loadFile').addEventListener('click', () => {
    window.ipcRender.invoke('openTrajectoryFile').then(() => {
        window.ipcRender.receive('getMissionData', (data) => {
            console.log(data.trajectory.position_ECEF);
        
            var greenLine = viewer.entities.add({
                polyline : {
                    positions : data.trajectory.position_ECEF,
                    material : Cesium.Color.SPRINGGREEN
                }
            });
        });
    });
})

console.log("App loaded.");