var ros = new ROSLIB.Ros({
    url : 'ws://10.129.141.97:9090'
  });

  ros.on('connection', function() {
    console.log("Connected");
    SetCam();
  });

  ros.on('error', function(error) {
    console.log("Error");
  });

  ros.on('close', function() {
    console.log("Closed");
  });

  function float2int (value) {
    return value | 0;
}

  var Scout_status = new ROSLIB.Topic({
    ros : ros,
    name : '/scout_status',
    messageType :'scout_msgs/ScoutStatus'
  });

  var cmdVel = new ROSLIB.Topic({
    ros : ros,
    name : '/cmd_vel',
    messageType : 'geometry_msgs/Twist'
  });
  
  Scout_status.subscribe(function(message) {
    document.getElementById("linear_v").innerHTML=message.linear_velocity.toFixed(2);
    document.getElementById("angular_v").innerHTML=message.angular_velocity.toFixed(2);
    var bat_voltage = float2int(message.battery_voltage);
    document.getElementById("battery_voltage").innerHTML= bat_voltage + ' [V]';
  });

  function SetCam(){
    console.log('set camera method')
    this.cameraViewer = new MJPEGCANVAS.Viewer({
        divID: 'video_streem',
        host: '10.129.141.97',
        width: 640,
        height: 480,
        topic: '/camera/color/image_raw',
        port: 9000,
    })
  }

  //function: execute when elements on page are loaded
  window.onload = function() {
    var Joy = new JoyStick("JoyContainer", {},function(stickData) {
      let y = -(stickData.x/100)*0.7;
      let x = (stickData.y/100)*0.7;

      Move(x,y,0,0,0,0);
    });

  }

  var leftbtn;
  var rightbtn;

  function StartLeft(){
    leftbtn=setInterval(function() {
      Move(0,0,0,0,0,0.5);
    }, 100);
  }
  function EndLeft(){
    clearInterval(leftbtn);
  }

  function StartRight(){
    rightbtn=setInterval(function() {
      Move(0,0,0,0,0,-0.5);
    }, 100);
  }
  function EndRight(){
    clearInterval(rightbtn);
  }

  function Move(lx,ly,lz,ax,ay,az){
    console.log(lx+" "+ly+" "+lz+" "+ax+" "+ay+" "+az);
    var twist = new ROSLIB.Message({
        linear : {
          x : lx,
          y : ly,
          z : lz
        },
        angular : {
          x : ax,
          y : ay,
          z : az
        }
      });

      cmdVel.publish(twist);
  }
