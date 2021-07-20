ws = new WebSocket("ws://"+ "192.168.43.187" +":8077");
            
var status_h1 = document.getElementById("status_h1");
var con_h1 = document.getElementById("con_h1");

var isWalkParamsLoaded = false;

var paramElements = [
    $("#set_init_x_offset"),
    $("#set_init_y_offset"),
    $("#set_init_z_offset"),
    $("#set_init_roll_offset"),
    $("#set_init_pitch_offset"),
    $("#set_init_yaw_offset"),
    $("#set_period_time"),
    $("#set_dsp_ratio"),
    $("#set_step_fb_ratio"),
    $("#set_x_move_amplitude"),
    $("#set_y_move_amplitude"),
    $("#set_z_move_amplitude"),
    $("#set_angle_move_amplitude"),
    $("#set_move_aim_on"),
    $("#set_balance_enable"),
    $("#set_balance_hip_roll_gain"),
    $("#set_balance_knee_gain"),
    $("#set_balance_ankle_roll_gain"),
    $("#set_balance_ankle_pitch_gain"),
    $("#set_y_swap_amplitude"),
    $("#set_z_swap_amplitude"),
    $("#set_arm_swing_gain"),
    $("#set_pelvis_offset"),
    $("#set_hip_pitch_offset"),
    $("#set_p_gain"),
    $("#set_i_gain"),
    $("#set_d_gain")
];

var btSubmitSettings = document.getElementsByClassName("submit_settings");

function setEnabledSettings(enabled){
    isWalkParamsLoaded = enabled;
    if(enabled){
        paramElements.forEach(element => {
            element.removeAttr('disabled');
        });
        for(let i = 0; i < btSubmitSettings.length; i++){
            btSubmitSettings[i].disabled = false;
        }
    }
    else{
        paramElements.forEach(element => {
            element.attr('disabled', 'disabled');
        });
        for(let i = 0; i < btSubmitSettings.length; i++){
            btSubmitSettings[i].disabled = true;
        }
    }
}
// setEnabledSettings(false);

function updateWalkParams(params){
    paramElements[0].value = params.init_x_offset;
    paramElements[1].value = params.init_y_offset;
    paramElements[2].value = params.init_z_offset;
    paramElements[3].value = params.init_roll_offset;
    paramElements[4].value = params.init_pitch_offset;
    paramElements[5].value = params.init_yaw_offset;
    paramElements[6].value = params.period_time;
    paramElements[7].value = params.dsp_ratio;
    paramElements[8].value = params.step_fb_ratio;
    paramElements[9].value = params.x_move_amplitude;
    paramElements[10].value = params.y_move_amplitude;
    paramElements[11].value = params.z_move_amplitude;
    paramElements[12].value = params.angle_move_amplitude;

    paramElements[13].checked = params.move_aim_on;
    paramElements[14].checked = params.balance_enable;

    paramElements[15].value = params.balance_hip_roll_gain;
    paramElements[16].value = params.balance_knee_gain;
    paramElements[17].value = params.balance_ankle_roll_gain;
    paramElements[18].value = params.balance_ankle_pitch_gain;
    paramElements[19].value = params.y_swap_amplitude;
    paramElements[20].value = params.z_swap_amplitude;
    paramElements[21].value = params.arm_swing_gain;
    paramElements[22].value = params.pelvis_offset;
    paramElements[23].value = params.hip_pitch_offset;
    paramElements[24].value = params.p_gain;
    paramElements[25].value = params.i_gain;
    paramElements[26].value = params.d_gain;

    setEnabledSettings(true);
}

ws.onopen = function (e){
    con_h1.innerHTML = "CONNECTED";
};

ws.onmessage = function (event){
    var obj = JSON.parse(event.data);
    
    if(obj.cmd == null) return;
    let cmd = obj.cmd;

    if(cmd == "update_walk_params"){
        updateWalkParams(obj.params);
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

    var result = '{"cmd":"set_walk_params","param":["'+ id.substring(4, id.length) +'","'+ el.value +'"] }';

    ws.send(result);

    console.log(id + el.value);
    return false;
}

function onSubmitCB(id){
    var el = document.getElementById(id);
    if(el == null) return false;

    var result = '{"cmd":"set_walk_params","param":["'+ id.substring(4, id.length) +'",'+ el.checked?'true':'false' +'] }';

    ws.send(result);

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
    setEnabledSettings(false);
    sendCmd("get_walk_params");
    console.log("get_walk_params");
}, false);


var collapsibles = [
    $("#settings"),
    $("#manual_ctrl")
];

var onShow = null; 

document.querySelector("#bt_show_settings").addEventListener("click", function(event) {
    event.preventDefault();

    if(onShow == null){
        collapsibles[0].collapse("show");
        onShow = collapsibles[0];
    }else if(onShow == collapsibles[0]){
        collapsibles[0].collapse("hide");
        onShow = null;
    }
    else{
        onShow.collapse("hide")
        collapsibles[0].collapse("show");
        onShow = collapsibles[0];
    }

}, false);

document.querySelector("#bt_show_manuctrl").addEventListener("click", function(event) {
    event.preventDefault();
    if(onShow == null){
        collapsibles[1].collapse("show");
        onShow = collapsibles[1];
    }else if(onShow == collapsibles[1]){
        collapsibles[1].collapse("hide");
        onShow = null;
    }
    else{
        onShow.collapse("hide")
        collapsibles[1].collapse("show");
        onShow = collapsibles[1];
    }
}, false);
