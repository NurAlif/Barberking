ws = new WebSocket("ws://"+ window.location.href.substring( 7, window.location.href.length - 1 ) +":8077");
            
var status_h1 = document.getElementById("status_h1");
var con_h1 = document.getElementById("con_h1");
var mode_h1 = document.getElementById("mode_h1");
var cm = 0;

var cf = false;
var cb = false;
var cl = false;
var cr = false;

ws.onopen = function (e){
    con_h1.innerHTML = "CONNECTED";
};

ws.onmessage = function (event){
    var obj = JSON.parse(event.data);
    var found = false;
    if(obj.count > 0){
        for(i = 0; i < obj.count; i++){
            if(obj.dets[i].class === 0) found = true;
        }
    }
    if(found){
        status_h1.innerHTML = "LOCK";
        status_h1.style.backgroundColor = "green";
    }else{
        status_h1.innerHTML = "LOST"
        status_h1.style.backgroundColor = "red";
    }
}

ws.onerror = function(err) {
    console.error('Socket encountered error: ', err.message, 'Closing socket');
    ws.close();
    con_h1.innerHTML = "ERR! DISCONNECTED";
    con_h1.style.backgroundColor = "red";
};

function onSubmit(id){
    var el = document.getElementById(id);
    if(el == null) return false;

    var newTune = "";

    if(id == 'xp'){
        newTune = "TPX" + el.value;
    }else if(id == 'xi'){
        newTune = "TIX" + el.value;
    }else if(id == 'xd'){
        newTune = "TDX" + el.value;
    }else if(id == 'yp'){
        newTune = "TPY" + el.value;
    }else if(id == 'yi'){
        newTune = "TIY" + el.value;
    }else if(id == 'yd'){
        newTune = "TDY" + el.value;
    }
    else if(id == 'os'){
        newTune = "M1" + el.value;
    }
    else if(id == 'ss'){
        newTune = "M2" + el.value;
    }
    else if(id == 'ms'){
        newTune = "M3" + el.value;
    }
    else if(id == 'td'){
        newTune = "M4" + el.value;
    }
    else if(id == 'tm'){
        newTune = "M5" + el.value;
    }
    else if(id == 'tc'){
        newTune = "M6" + el.value;
    }
    else if(id == 'mm'){
        newTune = "M7" + el.value;
    }
    else if(id == 'mc'){
        newTune = "M8" + el.value;
    }
    else if(id == 'ox'){
        newTune = "MQ" + el.value;
    }
    else if(id == 'oy'){
        newTune = "MW" + el.value;
    }
    else if(id == 'lx'){
        newTune = "ME" + el.value;
    }
    else if(id == 'ly'){
        newTune = "MR" + el.value;
    }

    ws.send(newTune);

    console.log(id + el.value);
    return false;
}


document.addEventListener("keydown", event => {
if (event.isComposing || event.keyCode === 38) {
    if(cf)  return;
    console.log("CF1");
    cf = true;
    ws.send("CF1");
}
if (event.isComposing || event.keyCode === 40) {
    if(cb)  return;
    console.log("CB1");
    cb = true;
    ws.send("CB1");
}
if (event.isComposing || event.keyCode === 37) {
    if(cl)  return;
    console.log("CL1");
    cl = true;
    ws.send("CL1");
}
if (event.isComposing || event.keyCode === 39) {
    if(cr)  return;
    console.log("CR1");
    cr = true;
    ws.send("CR1");
}
if (event.isComposing || event.keyCode === 32) {
    cm = 1
    ws.send("CS1");
    console.log("CS1");
    mode_h1.innerHTML = "AUTOMATIC";
    mode_h1.style.backgroundColor = "cyan";
}
if (event.isComposing || event.keyCode === 13) {
    cm = 0
    ws.send("CS0");
    console.log("CS0");
    mode_h1.innerHTML = "MANUAL";
    mode_h1.style.backgroundColor = "yellow";
}
if (event.isComposing || event.keyCode === 67) {
    ws.send("M0");
    console.log("M0");
}
});

document.addEventListener("keyup", event => {
if (event.isComposing || event.keyCode === 38) {
    console.log("CF0");
    cf = false;
    ws.send("CF0");
}
if (event.isComposing || event.keyCode === 40) {
    console.log("CB0");
    cb = false;
    ws.send("CB0");
}
if (event.isComposing || event.keyCode === 37) {
    console.log("CL0");
    cl = false;
    ws.send("CL0");
}
if (event.isComposing || event.keyCode === 39) {
    console.log("CR0");
    cr = false;
    ws.send("CR0");
}
});
