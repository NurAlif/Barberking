ws = new WebSocket("ws://"+ "192.168.43.187" +":8077");
            
var status_h1 = document.getElementById("status_h1");
var con_h1 = document.getElementById("con_h1");

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
    }

    ws.send(newTune);

    console.log(id + el.value);
    return false;
}


document.addEventListener("keydown", event => {
    if (event.isComposing || event.keyCode === 32) {
        
    }
    if (event.isComposing || event.keyCode === 13) {
        
    }
});

document.addEventListener("keyup", event => {
    
});

function sendCmd(cmd){
    var data = '{"cmd":"'+ cmd +'"}';
    ws.send(data);
}


document.querySelector("#bt_start").addEventListener("click", function(event) {
    event.preventDefault();
    sendCmd("torque_on");
    console.log("torqueon");
}, false);

document.querySelector("#bt_stop").addEventListener("click", function(event) {
    event.preventDefault();
    sendCmd("torque_off");
    console.log("torqueoff");
}, false);

document.querySelector("#bt_ena_walk").addEventListener("click", function(event) {
    event.preventDefault();
    sendCmd("ena_walk");
    console.log("ena_walk");
}, false);

document.querySelector("#bt_start_walk").addEventListener("click", function(event) {
    event.preventDefault();
    sendCmd("start_walk");
    console.log("start_walk");
}, false);

document.querySelector("#bt_stop_walk").addEventListener("click", function(event) {
    event.preventDefault();
    sendCmd("stop_walk");
    console.log("stop_walk");
}, false);

document.querySelector("#bt_get_walk_params").addEventListener("click", function(event) {
    event.preventDefault();
    sendCmd("get_walk_params");
    console.log("get_walk_params");
}, false);
