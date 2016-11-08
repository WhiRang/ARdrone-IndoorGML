var net    = require('net');
var arDrone = require('ar-drone');
var fs       = require('fs');
var drone = arDrone.createClient();
var pngStream = drone.getPngStream();
console.log('Connecting...');
drone.config('general:navdata_demo', 'TRUE');

var frameCounter = 0;

var lastFrameTime = 0;
var period = 100;
var lastPng;

function sleep(time, callback){
	var stop = new Dat().getTime();
	while(new Date().getTime() < stop + time){;}
	callback();
}

var client = net.connect({port: 4000, host:'localhost'},function(){
    console.log('Client connected');
   client.setNoDelay(true);
});

client.on('data', function(chunk){
	console.log('recv:' + chunk);
/*	
	if(String(chunk) == 0){
		eval("drone.takeoff();");
	}
	else if(String(chunk) == 1){ // turning right
		eval("drone.clockwise(0.6);");
	}
	else if(String(chunk) == 2){ // go front
		eval("drone.front(0.02);");
	}
	else if(String(chunk) == 3){ // turning left
		eval("drone.clockwise(-0.6);");
	}
	else if(String(chunk) == 4){ //left
		eval("drone.left(0.01);");
	}
	else if(String(chunk) == 5){ // right
		eval("drone.right(0.01);");
	}
	else if(String(chunk) == 6){ // landing
		eval("drone.land();");
	}
	else if(String(chunk) == 7){ // stop
		eval("drone.stop();");
	}
	else if(String(chunk) == 8){ // up
		eval("drone.up(0.3);");
	}
	*/
});

pngStream
  .on('error', console.log)
  .on('data', function(pngBuffer) {
  var now = (new Date()).getTime();
  if(now - lastFrameTime > period) {
   frameCounter++;
   lastFrameTime = now;
   lastPng = pngBuffer;
    fs.writeFile('./photo/frame' + frameCounter + '.png', lastPng, function(err) {
        if (err) {
          console.log('Error saving PNG: ' + err);
        }
      else{
         var name = 'frame' + frameCounter + '.png'
         console.log(name);
         client.write(name);
      }
      });
     
    
  }
  });