var isKeyboardHasControl = true;

var cf = false;
var cb = false;
var cl = false;
var cr = false;

document.addEventListener("keydown", event => {
    event.preventDefault();
    if (event.isComposing || event.keyCode === 38) {
        if(cf)  return;
        cf = true;
    }
    if (event.isComposing || event.keyCode === 40) {
        if(cb)  return;
        cb = true;
    }
    if (event.isComposing || event.keyCode === 37) {
        if(cl)  return;
        cl = true;
    }
    if (event.isComposing || event.keyCode === 39) {
        if(cr)  return;
        cr = true;
    }
});

document.addEventListener("keyup", event => {
    event.preventDefault();
    if (event.isComposing || event.keyCode === 38) {
        cf = false;
    }
    if (event.isComposing || event.keyCode === 40) {
        cb = false;
    }
    if (event.isComposing || event.keyCode === 37) {
        cl = false;
    }
    if (event.isComposing || event.keyCode === 39) {
        cr = false;
    }
});

function getKeyboardControlVector(){
    var vec = new Vector2(0.0, 0.0); 
    if(cf)
        vec.y += 0.5;
    if(cb)
        vec.y -= 0.5;
    if(cl)
        vec.x -= 0.5;
    if(cr)
        vec.x += 0.5;

    return vec;
}