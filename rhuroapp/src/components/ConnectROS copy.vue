<template>
  <v-container>
     <div class="text-center">
        <v-text-field v-model="txtURLServer" label="ROSBridge address" clearable></v-text-field>
        <v-slider v-model="value1" label="Value 1"></v-slider>
        <v-slider v-model="value2" label="Value 2"></v-slider>

        <v-alert dense text type="success" v-if="this.$gConnectedROS">Connected to ROS</v-alert>
        <v-btn depressed color="error" @click="disconectROS()" v-if="this.$gConnectedROS">Disconnect </v-btn>
        <v-btn depressed color="info" @click="connectROS()" v-else>Connect </v-btn>
        <v-btn @click="publishROS()" v-if="this.$gConnectedROS">Publish </v-btn>

      </div>
  </v-container>
  
</template>

<script>
 import ROSLIB from 'roslib';
  export default {
    name: 'ConnectROS',
    data: () => ({
          value1: 1,
          value2: 1,
          txtURLServer: null,
    }), 
    mounted: function () {
      this.txtURLServer=this.$gTxtURLServer
        
    },
      methods: {
        setConsole: function (valueConsole) {
          alert(valueConsole);
        },
         globalFunction1: function () {   
            console.log('globalFunction')
        },
        connectROS: function () {
          console.log(this.$gTxtURLServer)
          this.$gROS = new ROSLIB.Ros({
            url : this.txtURLServer
          });

          this.$gROS.on('connection', () => {
            this.$gConnectedROS = true;
          });

          this.$gROS.on('error', (error) => {
              alert('Something went wrong when trying to connect' + error)
              console.log(error)
          });
          this.$gROS.on('close', () => {
                this.$gConnectedROS = false
                console.log('Connection to ROSBridge was closed!')
                clearInterval(this.pubInterval)
          });
        },
        disconectROS: function(){
          this.$gROS.close();
          this.$gConnectedROS=false;
        },
        publishROS: function () {
              let topic = new ROSLIB.Topic({
                  ros: this.$gROS,
                  name: '/cmd_vel',
                  messageType: 'geometry_msgs/Twist'
              })
              let message = new ROSLIB.Message({
                  linear: { x: this.value1, y: 0, z: 0, },
                  angular: { x: 0, y: 0, z: this.value2, },
              })
              topic.publish(message)
          }
        }

    
    
  }
</script>
