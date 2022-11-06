const dialog = require('electron').dialog

function openMissionFile() {
    // construct the select file dialog 
    dialog.showOpenDialog({
        properties: ['openFile']
    })
    .then(function(fileObj) {
        if (!fileObj.canceled) {
            mainWindow.webContents.send('FILE_OPEN', fileObj.filePaths);
            functions.openMissionFile(fileObj.filePaths);
        }
    })
    .catch(function(err) {
        console.error(err)  
    })
}

module.exports = {openMissionFile};