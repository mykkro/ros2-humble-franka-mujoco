let rosInstance = null;

function onModelLoadingFinished(data) {
    console.log("Robot model loading finished!", data)
    ROSInit();
}

function ROSInit() {
            // Connecting to ROS
        // -----------------
        var ros = new ROSLIB.Ros();
        rosInstance = ros;
      
        // If there is an error on the backend, an 'error' emit will be emitted.
        ros.on('error', function(error) {
          document.getElementById('connecting').style.display = 'none';
          document.getElementById('connected').style.display = 'none';
          document.getElementById('closed').style.display = 'none';
          document.getElementById('error').style.display = 'inline';
          console.log(error);
        });
      
        // Find out exactly when we made a connection.
        ros.on('connection', function() {
          console.log('Connection made!');
          document.getElementById('connecting').style.display = 'none';
          document.getElementById('error').style.display = 'none';
          document.getElementById('closed').style.display = 'none';
          document.getElementById('connected').style.display = 'inline';
        });
      
        ros.on('close', function() {
          console.log('Connection closed.');
          document.getElementById('connecting').style.display = 'none';
          document.getElementById('connected').style.display = 'none';
          document.getElementById('closed').style.display = 'inline';
        });
      
        // Create a connection to the rosbridge WebSocket server.
        ros.connect('ws://localhost:9090');
      
        // Publishing a Topic
        // ------------------
      
        var listener = new ROSLIB.Topic({
          ros : ros,
          name : '/joint_states',
          messageType : 'sensor_msgs/msg/JointState'
        });
      
      
        // Then we add a callback to be called every time a message is published on this topic.
        listener.subscribe(function(message) {
            const positions = message.position;
            const names = message.name
            //console.log('Received positions:', positions, names);

            // update joints in viewer based on incoming topic
            for(var i=0; i<names.length; i++) {
                var name = names[i];
                var pos = positions[i];
                viewer.setJointValue(name, pos)
            }
      
          // If desired, we can unsubscribe from the topic as well.
          // listener.unsubscribe();
        });
      

        return ros;
}