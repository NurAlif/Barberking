

var fieldItem = function(time, value){
    this.timeStamp = time,
    this.value = value,
    this.x = 0
};

var graphLine = function(color,len){
    this.queue = [],
    this.color = color,
    this.len = len,
    this.addItem = function(item){
        var queue = this.queue;
        if(queue.length > len)queue.shift();
        queue.push(item);
    }
};

var GraphMonitor = function(el){
    this.canvas = el,
    this.height = 300,
    this.width = 500,
    this.length = 30,
    this.scaleX = 1,
    this.scaleY = 0,
    this.lines = [],
    this.build = function (){
        var ctx = this.canvas.getContext("2d");
        this.canvas.setAttribute("height",this.height);
        this.canvas.setAttribute("width", this.width);
        this.lines[0] = new graphLine("red");
        this.lines[1] = new graphLine("blue");

        
    },
    this.updateAllLine = function(){
        var ratioX = this.scaleX * this.width;
        var ratioY = this.scaleY * this.height;

        var ctx = this.canvas.getContext("2d");
        
        ctx.lineWidth = 4;

        for(var l = 0; l < this.lines.length; l++){
            var queue = this.lines[l].queue;
            var len = this.lines[l].queue.length;
            
            if(queue.len <= 0) break;
            // first
            ctx.beginPath();
            ctx.strokeStyle = this.lines[l].color;

            var item = queue[len-1];
            var posX = this.width;
            var posY = ratioY * item.value;
            ctx.moveTo(posX, posY);

            for (var i = len-2; i >= 0; i--){
                var item = queue[i];
                var past = queue[i-1];
                var posX = past.x - ratioX * item.timeStamp;
                var posY = ratioY * item.value;
                ctx.lineTo(posX, posY);
                console.log(posX +"   "+ posY);
                item.x = posX;

            }
            ctx.stroke();

            ctx.closePath();
        }
    },
    this.addItems = function(str){
        var obj = JSON.parse(str.replace('\\', ''));
        var timestamp = obj.timestamp;

        this.lines[0].addItem(new fieldItem(timestamp, obj.input_pitch));
        this.lines[1].addItem(new fieldItem(timestamp, obj.corr_pitch));
    }
};

