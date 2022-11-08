const contextBridge = require('electron').contextBridge;
const ipcRenderer = require('electron').ipcRenderer;
const child_process = require("child_process");

function runExecutable(executablePath, args) {
    var child = child_process.execFile;
    var command = executablePath + " " + args;
    console.log(command);
    child(command, function(err, data) {
        if(err){
            console.error(err);
            return;
        }
        console.log(data.toString());
    });
};

// White-listed channels.
const ipc = {
    'render': {
        // From render to main.
        'send': [],
        // From main to render.
        'receive': [],
        // From render to main and back again.
        'sendReceive': [
            'dialog:openFile' // Channel name
        ]
    }
};

// Exposed protected methods in the render process.
contextBridge.exposeInMainWorld(
    // Allowed 'ipcRenderer' methods.
    'ipcRender', {
        // From render to main.
        send: (channel, args) => {
            let validChannels = ipc.render.send;
            if (validChannels.includes(channel)) {
                ipcRenderer.send(channel, args);
            }
        },
        // From main to render.
        receive: (channel, listener) => {
            let validChannels = ipc.render.receive;
            if (validChannels.includes(channel)) {
                // Deliberately strip event as it includes `sender`.
                ipcRenderer.on(channel, (event, ...args) => listener(...args));
            }
        },
        // From render to main and back again.
        invoke: (channel, args) => {
            let validChannels = ipc.render.sendReceive;
            if (validChannels.includes(channel)) {
                return ipcRenderer.invoke(channel, args);
            }
        }
    }
);

window.addEventListener('DOMContentLoaded', () => {
    // do stuff when window loads
    var btn = document.getElementById("runSim");
    btn.addEventListener('click', () => {runExecutable("bin\\rocket.exe","test.srocket");});
});
