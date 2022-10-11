<template>
  <v-container>
       
      <div v-if="vbConnectROS"  class="text-center">
         <h3>Touch the circle</h3> 
            <v-divider></v-divider>
            <br/>
            <div class="box position lighten-2" v-hammer:pan="pan" >
              <div class="circle" v-hammer:tap="onTap" v-hammer:press="onPress" ></div>
            </div>
            
      </div >
      <div v-else class="text-center">
          <v-progress-circular
            indeterminate
            :size="70"
            :width="7"
            color="red">
          </v-progress-circular>
          <br/><br/><br/>
          <v-text-field v-model="txtURLServer" label="ROSBridge address" clearable></v-text-field>
          <v-btn  color="teal lighten-4" @click="connectROS()" >Reconnect </v-btn>
          <br/><br/>
          <v-textarea
              readonly="true"
              name="input-7-1"
              label="Console:"
              v-model="txtConsole"
              hint="Result request"
          ></v-textarea>
      </div>

    <v-snackbar
      v-model="snackbar"
      color="red accent-2" >
      {{ text }}

      <template v-slot:action="{ attrs }">
        <v-btn
          color="pink"
          text
          v-bind="attrs"
          @click="snackbar = false"
        >
          Close
        </v-btn>
      </template>
    </v-snackbar>
    <div v-if="vbConnectROS" >
              <v-btn
                  color="cyan darken-4"
                  dark
                  absolute
                  top
                  right
                  @click="moveEndEffector()"
                >
                  <v-icon>mdi-human-handsdown </v-icon>
                </v-btn>
      </div>      
  </v-container>
</template>
<script> 
  import Vue from 'vue';
  import { VueHammer } from 'vue2-hammer';
  import ROSLIB from 'roslib';
  
  Vue.use(VueHammer)
  
 export default {
    name: 'TestExercise',
    data: () => ({
        txtConsole:null,
        txtURLServer: null,
        ros: null,
        topic: null,
        snackbar: false,
        text: null,
        topicUserTracking: null,
        vbConnectROS: false,
    }),
    mounted: function () {
        this.txtURLServer=this.$gTxtURLServer;
        this.ros = new ROSLIB.Ros({
            url : this.txtURLServer
          });
          this.ros.on('connection', () => {
            console.log('Ok connection');
            this.vbConnectROS=true;
          });
          this.ros.on('error', (error) => {
              
              this.txtConsole='Something went wrong when trying to connect ROSBridge' + JSON.stringify(error)+' '+this.txtURLServer;
              console.log(this.txtConsole);
              alert(this.txtConsole);
              this.vbConnectROS=false;
          });
          this.ros.on('close', () => {
              console.log('Connection to ROSBridge was closed!')
              clearInterval(this.pubInterval)
          });

          this.topic = new ROSLIB.Topic({
                  ros: this.ros,
                  name: '/huro_app_circle', 
                  messageType: 'std_msgs/Bool'
              });
          this.topicUserTracking = new ROSLIB.Topic({
                  ros: this.ros,
                  name: '/hu_app_user_tracking', 
                  messageType: 'std_msgs/Bool'
              });
          

    },
    methods: {
      connectROS2: function () {
          alert('Hola');
      },
      onTap: function(e){
        this.txtConsole= "onTap: (" + e.center.x+ ","+e.center.y+")";
        this.snackbar = true;
        this.text='the UR will make a move! E1';
        let message = new ROSLIB.Message({data: true})
        this.topic.publish(message)
      },
      onPress: function(e){
        this.txtConsole= "onPress: (" + e.center.x+ ","+e.center.y+")";
        this.snackbar = true;
        
        let message = new ROSLIB.Message({data: true})
        this.topic.publish(message)
        this.text='the UR will make a move! E2';
      },
      connectROS: function () {
          this.ros = new ROSLIB.Ros({
            url : this.txtURLServer
          });
          this.ros.on('connection', () => {
            this.vbConnectROS=true
          });
          this.ros.on('error', (error) => {
            this.txtConsole='Something went wrong when trying to connect ROSBridge' + JSON.stringify(error)+' '+this.txtURLServer;
            alert(this.txtConsole);
            alert(JSON.stringify(error));
               
          });
          this.ros.on('close', () => {
              this.vbConnectROS=false;
          });

          this.topic = new ROSLIB.Topic({
                  ros: this.ros,
                  name: '/huro_app_circle', 
                  messageType: 'std_msgs/Bool'
              });
          this.topicUserTracking = new ROSLIB.Topic({
                  ros: this.ros,
                  name: '/hu_app_user_tracking', 
                  messageType: 'std_msgs/Bool'
              });
        },
        moveEndEffector: function () {
            this.text='the UR will make a move! [User Tracking]';
            let message = new ROSLIB.Message({data: true})
            this.topicUserTracking.publish(message)
        },
    } 
  }
</script>
<style>
 .circle {
    touch-action: none;
    cursor: grab;
    width: 220px;
    height: 220px;
    background: #009688;
    border-radius: 50%;
  }

 .box {
    touch-action: none;
    cursor: grab;
    width: 100%;
    height: 100%;
    
    border-radius: 1%;
  }
  .position {
     display: flex;
    align-items: center;
    justify-content: center;
    align:center;
    align-text: center;
  }
</style>