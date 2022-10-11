<template>
  <v-container>
     <div class="text-center">
        <v-text-field v-model="txtURLServer" label="ROSBridge address" clearable></v-text-field>
        <v-alert dense text type="success" v-if="connected">Save</v-alert>
        <v-btn depressed color="error" @click="disconectROS()" v-if="connected">Disconnect </v-btn>
        <v-btn depressed color="info" @click="connectROS()" v-else>Save </v-btn>
        <v-btn @click="publishROS()" v-if="connected">Publish </v-btn>
      </div>
  </v-container>
</template>

<script>
  import ROSLIB from 'roslib';
  export default {
    name: 'ConnectROS',
    data: () => ({
          ros: null,
          connected: false,
          txtURLServer: null,
    }), 
    mounted: function () {
        this.txtURLServer=this.$gTxtURLServer
    },
      methods: {
        connectROS: function () {
          this.ros = new ROSLIB.Ros({
            url : this.txtURLServer
          });
          this.ros.on('connection', () => {
            this.connected=true
          });
          this.ros.on('error', (error) => {
              alert('Something went wrong when trying to connect' + error)
              alert(JSON.stringify(error))
              console.log(error)
          });
          this.ros.on('close', () => {
              this.connected=false
              console.log('Connection to ROSBridge was closed!')
              clearInterval(this.pubInterval)
          });
        },
        disconectROS: function(){
          this.ros.close();
          this.connected=false
        },
        publishROS: function () {
              let topic = new ROSLIB.Topic({
                  ros: this.ros,
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
