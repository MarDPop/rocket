

document.getElementById('runSim').addEventListener('click', () => {
    window.ipcRender.invoke('dialog:openFile')
        .then((paths) => {
            if (paths === undefined) { return } // Dialog was cancelled

            let result = '';

            for (let path of paths) {
                result += '<li>' + path + '</li>';
            }

            document.getElementById('test').innerHTML = result;
        })
})

document.getElementById('loadFile').addEventListener('click', () => {
    window.ipcRender.invoke('dialog:openFile')
        .then((paths) => {
            if (paths === undefined) { return } // Dialog was cancelled

            console.log("loading " + paths); 

            
        })
})

console.log("App loaded.");