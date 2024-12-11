// app.js

// default VID and PID for Pi Pico
let vid = 0x2e8a;
let pid = 0x000a;
let baudRate = 115200;

let maxDistance = 2560;
let distScaleDenom = maxDistance / 256;

let imageWidth = 480; // this is the width and height of the displayed image

var usb = null;
var comm;
var reader = null;
var writer = null;
var deviceInitialized = false;
var inBuffer = '';
var canvas;
var ctx;
var canvas2;
var ctx2;
var enc = new TextEncoder();
var dec = new TextDecoder();
var objLines = [];
// main grid of distances
var grid = [
    0, 0, 0, 0, 0, 0, 0, 0,
    0, 0, 0, 0, 0, 0, 0, 0,
    0, 0, 0, 0, 0, 0, 0, 0,
    0, 0, 0, 0, 0, 0, 0, 0,
    0, 0, 0, 0, 0, 0, 0, 0,
    0, 0, 0, 0, 0, 0, 0, 0,
    0, 0, 0, 0, 0, 0, 0, 0,
    0, 0, 0, 0, 0, 0, 0, 0
    ];

// grid of distances for any secondary object
var grid2 = [
    0, 0, 0, 0, 0, 0, 0, 0,
    0, 0, 0, 0, 0, 0, 0, 0,
    0, 0, 0, 0, 0, 0, 0, 0,
    0, 0, 0, 0, 0, 0, 0, 0,
    0, 0, 0, 0, 0, 0, 0, 0,
    0, 0, 0, 0, 0, 0, 0, 0,
    0, 0, 0, 0, 0, 0, 0, 0,
    0, 0, 0, 0, 0, 0, 0, 0
    ];

// These are indices to the grid and results arrays,
// used for meshing the #Obj data per 4x4 sub-capture, into the 8x8 grid
var gridIdxCap0 = [8,9,12,13,24,25,28,29,40,41,44,45,56,57,60,61];
var gridIdxCap1 = [10,11,14,15,26,27,30,31,42,43,46,47,58,59,62,63];
var gridIdxCap2 = [0,1,4,5,16,17,20,21,32,33,36,37,48,49,52,53];
var gridIdxCap3 = [2,3,6,7,18,19,22,23,34,35,38,39,50,51,54,55];
var gridIdxCapArr = [gridIdxCap0, gridIdxCap1, gridIdxCap2, gridIdxCap3];
var resIdxCapAll = [6,15,7,16,4,13,5,14,2,11,3,12,0,9,1,10];

// functions
function ClearGrid() {
    for (let i = 0; i < grid.length; i++) {
        grid[i] = maxDistance;
    }
}

function ClearGrid2() {
    for (let i = 0; i < grid2.length; i++) {
        grid2[i] = maxDistance;
    }
}

// Function to process the 4 lines of #Obj data which is comma separated
function ProcessObjLines() {
    ClearGrid();
    ClearGrid2();
    // loop through the 4 lines of #Obj data to mesh all subcaptures into the 8x8 grid
    for (let i = 0; i < objLines.length; i++) {
        let subcapture_parts = objLines[i].split(',');
        let subcapture_identifier = parseInt(subcapture_parts[2]) & 0x03;
        subcapture_parts.splice(0, 6); // only distance and confidence values remaining
        // perform the meshing of the current sub-capture
        for (let j = 0; j < 16; j++) {
            let distIdx = resIdxCapAll[j] * 2;
            let confidenceIdx = distIdx + 1;
            let gridIdx = gridIdxCapArr[subcapture_identifier][j];
            grid[gridIdx] = subcapture_parts[distIdx];
            let confidence = subcapture_parts[confidenceIdx];
            if (grid[gridIdx] >= maxDistance) { // sanity checks
                grid[gridIdx] = maxDistance;
            }
            if (grid[gridIdx] == 0) { // if a value is zero, set it to maxDistance
                grid[gridIdx] = maxDistance;
            }
            if (confidence < 20) { // if confidence is low, set to maxDistance
                grid[gridIdx] = maxDistance;
            }
            // scale the range to 0-255
            grid[gridIdx] = Math.floor(grid[gridIdx] / distScaleDenom);

            // second object
            distIdx = (resIdxCapAll[j] + 18) * 2;
            confidenceIdx = distIdx + 1;
            grid2[gridIdx] = subcapture_parts[distIdx];
            confidence = subcapture_parts[confidenceIdx];
            if (grid2[gridIdx] >= maxDistance) {
                grid2[gridIdx] = maxDistance;
            }
            if (grid2[gridIdx] == 0) {
                grid2[gridIdx] = maxDistance;
            }
            if (confidence < 20) {
                grid2[gridIdx] = maxDistance;
            }
            grid2[gridIdx] = Math.floor(grid2[gridIdx] / distScaleDenom);
        }
    }
    let image = ctx.createImageData(8, 8);
    // build a grayscale image from the grid data
    for (let i = 0; i < 64; i++) {
        let val = grid[i];
        image.data[i*4] = val;
        image.data[i*4+1] = val;
        image.data[i*4+2] = val;
        image.data[i*4+3] = 255;
    }
    createImageBitmap(image).then((bmp) => {
        ctx.drawImage(bmp, 0, 0, 8, 8, 0, 0, imageWidth, imageWidth); // scaled to larger square
    });
    // second object
    let image2 = ctx2.createImageData(8, 8);
    for (let i = 0; i < 64; i++) {
        let val = grid2[i];
        image2.data[i*4] = val;
        image2.data[i*4+1] = val;
        image2.data[i*4+2] = 255;
        image2.data[i*4+3] = 255;
    }
    createImageBitmap(image2).then((bmp) => {
        ctx2.drawImage(bmp, 0, 0, 8, 8, 0, 0, imageWidth, imageWidth);
    });
}

async function Comm()
{
    var filter = {};
    filter["usbVendorId"] = vid;
    filter["usbProductId"] = pid;

    if ('serial' in navigator) {
        try {
            comm = await navigator.serial.requestPort("usbVendorId" in filter ? { filters: [filter] } : {});
            await comm.open({ baudRate: baudRate,
                              dataBits: 8,
                              parity: "none",
                              stopBits: 1,
                              bufferSize: 1024 });
            reader = comm.readable.getReader();
            writer = comm.writable.getWriter();

            setTimeout(Receive, 10);

        } catch(err) {
            console.log('Error: ', err);
        }
    } else {
        document.getElementById("infoSpace").innerHTML +=
`The Web serial API needs to be enabled in your browser thru:
   - <a href=edge://flags/#enable-experimental-web-platform-features>edge://flags/#enable-experimental-web-platform-features</a>
   - <a href=chrome://flags/#enable-experimental-web-platform-features>chrome://flags/#enable-experimental-web-platform-features</a>
   - <a href=opera://flags/#enable-experimental-web-platform-features>opera://flags/#enable-experimental-web-platform-features</a>
`;
    }
}

async function Send(str)
{
    if (usb) {
        // not implemented
    } else {
        await writer.write(enc.encode(str).buffer);
    }
}

async function Receive()
{
    var result;
    var objStartIdx = -1;
    var newLineIdx = -1;
    var notFinished = true;
    var str = '';
    if (usb) {
        // not implemented
    } else {
        result = await reader.read();
        str = dec.decode(result.value);
    }
    // append to the inBuffer
    inBuffer += str;
    while (notFinished) {
        // find the first #Obj in the inBuffer
        objStartIdx = inBuffer.indexOf('#Obj');
        // find the first newline in the inBuffer, starting from the objStartIdx
        newLineIdx = inBuffer.indexOf('\n', objStartIdx);
        // if the objStartIdx is found and the newLineIdx is found
        if (objStartIdx != -1 && newLineIdx != -1) {
            // extract the string from objStartIdx to newLineIdx
            str = inBuffer.substring(objStartIdx, newLineIdx+1);
            // remove the string from inBuffer
            inBuffer = inBuffer.substring(newLineIdx+1);
            //document.getElementById("infoSpace").innerText += str;
            if (objLines.length > 0) {
                objLines.push(str);
            } else {
                let str_parts = str.split(',');
                let subcapture_number = parseInt(str_parts[2]) & 0x03;
                if (subcapture_number == 0) {
                    objLines.push(str);
                }
            }
            //document.getElementById("infoSpace").innerText += 'num objLines = ' + objLines.length + '\n';
            if (objLines.length == 4) {
                ProcessObjLines();
                objLines = [];
            }
        }
        else {
            notFinished = false;
        }
    }
    setTimeout(Receive, 10);
}

// Page load
document.addEventListener('DOMContentLoaded', function() {
    //
});

// Start button click
document.getElementById('btnStart').addEventListener('click', function() {
    if (!deviceInitialized) {
        Send('d\n'); // disable the remote device
        Send('e\n'); // start the remote device
        Send('l\n'); // load the factory calibration
        deviceInitialized = true;
    }
    Send('m\n'); // start the measurement stream
});

// Stop button click
document.getElementById('btnStop').addEventListener('click', function() {
    Send('s\n');
});

// body Load() function
function Load() {
    deviceInitialized = false;
    Comm();
    // first object
    canvas = document.createElement('canvas');
    canvas.height = imageWidth+5;
    canvas.width = imageWidth+5;
    document.getElementById('article').appendChild(canvas);
    ctx = canvas.getContext('2d');
    ctx.fillStyle = 'solid';
    ctx.strokeStyle = '#ECD018';
    ctx.lineWidth = 5;
    ctx.lineCap = 'round';
    
    // secondary object
    canvas2 = document.createElement('canvas');
    canvas2.height = imageWidth+5;
    canvas2.width = imageWidth+5;
    document.getElementById('article2').appendChild(canvas2);
    ctx2 = canvas2.getContext('2d');
    ctx2.fillStyle = 'solid';
    ctx2.strokeStyle = '#ECD018';
    ctx2.lineWidth = 5;
    ctx2.lineCap = 'round';
}

