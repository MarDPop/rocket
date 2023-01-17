const electron = require('electron');
const BrowserWindow = electron.BrowserWindow;
const Menu = electron.Menu
const app = electron.app;

const dialog = require('electron').dialog;
const ipcMain = require('electron').ipcMain;
const child_process = require("child_process");
const path = require('path');
const global = require('./lib/global');
const fs = require('fs');
const { contextIsolated } = require('process');

var mainWindow = null;
var dialogWindow = null;

// This method will be called when Electron has finished
// initialization and is ready to create browser windows.
function createWindow() {
    // Create the browser window
    mainWindow = new BrowserWindow({
        width: 1024, 
        height: 720,
        fullscreen: false,
        webPreferences: {
            preload: path.join(__dirname, 'preload.js'),
            devTools: true,
            sandbox: false
        }
    });

    // and load the index.html of the app.
    mainWindow.loadURL( path.join(__dirname, 'index.html'));

    const templateMenu = [
        {
           label: 'File',
           submenu: [
                {
                    label: 'Open Mission File',
                    click() {
                        let options = {
                            properties: ['openFile']
                        };
                        dialog.showOpenDialog(mainWindow, options)
                        .then((result) => {
                            // Bail early if user cancelled dialog
                            if (result.canceled) { return }

                            return global.DATA.load(result.filePaths);
                        })
                    }
                },
                {
                    label: 'Save Mission File',
                    role: 'Save'
                },
                {
                    label: 'Options',
                    role: 'Options'
                },
                {
                    label: 'Exit',
                    click() {
                        app.quit()
                    } 
                }
            ]
        },
        {
            label: 'Rocket',
            submenu: [
                {
                    label: 'Design',
                    role: 'Rocket Design'
                },
                {
                    label: 'Simulation',
                    role: 'Rocket Simulation'
                },
                {
                    label: 'Analysis',
                    role: 'Rocket Analysis'
                }
            ]
        },
        {
            label: 'View',
            submenu: [
                {
                    label: 'Rocket Perspective',
                    role: 'Rocket Design'
                },
                {
                    label: 'Earth Perspective',
                    role: 'Rocket Simulation'
                },
                {
                    label: 'Analysis Tools',
                    click() {
                        mainWindow.webContents.openDevTools();
                    }
                },
                {
                    label: 'Dev Tools',
                    click() {
                        mainWindow.webContents.openDevTools();
                    }
                }
            ]
        }
    ];

    const menu = Menu.buildFromTemplate(templateMenu);
    Menu.setApplicationMenu(menu);

    // Returned when the window is closed.
    mainWindow.on('closed', function() {
        // Dereference the window object. Usually you would store windows
        // in an array if your app supports multi windows. This is the time
        // when you should delete the corresponding element.
        mainWindow = null;
    });

    mainWindow.show();
}

// This method will be called when Electron has finished
// initialization and is ready to create browser windows.
// Some APIs can only be used after this event occurs.
app.whenReady().then(() => {
    createWindow()

    app.on('activate', function () {
        // On macOS it's common to re-create a window in the app when the
        // dock icon is clicked and there are no other windows open.
        if (BrowserWindow.getAllWindows().length === 0) createWindow()
    });    
});

// On a PC, the app will quit when we close all windows.
// On a Mac, applications must be explicitly closed.
app.on('window-all-closed', function() {
    if (process.platform != 'darwin') {
        app.quit();
    }
});

ipcMain.handle('runSim', (event, data) => {
    console.log("running sim.")
    global.writeRocketFile(data);
    var data = child_process.spawnSync("./bin/rocket.exe", ['./data/rocket.srocket', './data/rocket.traj']);

    if(data.error){
        console.log(data.stderr);
        return null;
    }

    console.log(data.toString());

    global.DATA.trajectory.load('./data/rocket.traj');
    return [global.DATA.trajectory.times,global.DATA.trajectory.position_ECEF,global.DATA.trajectory.orientation_ECEF];
});

ipcMain.handle('getFile', () => {
    let options = {
        properties: ['openFile']
    };

    return dialog.showOpenDialog(mainWindow, options)
        .then((result) => {
            // Bail early if user cancelled dialog
            if (result.canceled) { return }

            return fs.readFile(result.filePaths[0],mimeType,(err,contents)=>{
                if(err){
                   console.log(err);
                   return;
                }
                return contents;
              })
        })
})

ipcMain.handle('openTrajectoryFile', () => {
    let options = {
        properties: ['openFile']
    };

    return dialog.showOpenDialog(mainWindow, options).then((result) => {
        if (result.canceled) { return }
        global.DATA.trajectory.load(result.filePaths[0]);
        return [global.DATA.trajectory.times,global.DATA.trajectory.position_ECEF,global.DATA.trajectory.orientation_ECEF];
    });
})

ipcMain.handle('getMissionData', () => {
    return [global.DATA.trajectory.times,global.DATA.trajectory.position_ECEF,global.DATA.trajectory.orientation_ECEF];
});

