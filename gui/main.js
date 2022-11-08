const electron = require('electron');
const BrowserWindow = electron.BrowserWindow;
const Menu = electron.Menu
const app = electron.app;

const dialog = require('electron').dialog;
const ipcMain = require('electron').ipcMain;

const path = require('path');
const global = require('./lib/global');

var mainWindow = null;

// This method will be called when Electron has finished
// initialization and is ready to create browser windows.
function createWindow() {
    // Create the browser window
    mainWindow = new BrowserWindow({
        width: 1024, 
        height: 640,
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

                            return global.openMissionFile(result.filePaths);
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

ipcMain.handle('dialog:openFile', () => {
    let options = {
        properties: ['openFile']
    };

    return dialog.showOpenDialog(mainWindow, options)
        .then((result) => {
            // Bail early if user cancelled dialog
            if (result.canceled) { return }

            return result.filePaths;
        })
})