<template>
  <v-container>
    <p><b>Gestures:</b>tap, pan, pinch, press, rotate, swipe</p>
    <p><b>Directions:</b>up, down, left, right, horizontal, vertical, all</p>
    <div class="box grey lighten-2" v-hammer:pan="pan" >Pan</div> <br/>
    <v-text-field  elevation="2" fab v-model="txtConsole" label="Console" clearable></v-text-field>
    <div class="box grey lighten-2" v-hammer:pinch="pinch">Pinch</div><br/>
    <v-text-field  elevation="2" fab v-model="txtConsole" label="Console" clearable></v-text-field>
    <div class="box grey lighten-2" v-hammer:press="onPress">Press</div><br/>
    <v-text-field  elevation="2" fab v-model="txtConsole" label="Console" clearable></v-text-field>
    <div class="box grey lighten-2" v-hammer:tap="onTap">Tap</div><br/>
   
    <v-text-field  elevation="2" fab v-model="txtConsole" label="Console" clearable></v-text-field>
  </v-container>
</template>
<script>
  import ROSLIB from 'roslib';
  import Vue from 'vue'
  import { VueHammer } from 'vue2-hammer'
  Vue.use(VueHammer)

 export default {
    name: 'TestExercise',
    data: () => ({
        txtConsole:null,
        txtURLServer: null,
        ros: null
    }),
    mounted: function () {
        this.txtURLServer=this.$gTxtURLServer;
        this.ros = new ROSLIB.Ros({
            url : this.txtURLServer
          });
          this.ros.on('connection', () => {
            console.log('Ok connection')
          });
          this.ros.on('error', (error) => {
              alert('Something went wrong when trying to connect' + error)
              console.log(error)
          });
          this.ros.on('close', () => {
              console.log('Connection to ROSBridge was closed!')
              clearInterval(this.pubInterval)
          });
    },
    methods: {
      pinch: function(e){
        this.txtConsole= "pinch: (" + e.center.x+ ","+e.center.y+")";
        console.log(e);
      },
      onTap: function(e){
        this.txtConsole= "onTap: (" + e.center.x+ ","+e.center.y+")";
        console.log(e);
      },
       pan: function(e){
        this.txtConsole= "pan: (" + e.center.x+ ","+e.center.y+")";
        console.log(e);

        let topic = new ROSLIB.Topic({
                  ros: this.ros,
                  name: '/ha_touch_pan', 
                  messageType: 'geometry_msgs/Twist'
              })
              let message = new ROSLIB.Message({
                  linear: { x: e.center.x, y: e.center.y, z: 0, },
                  angular: { x: 0, y: 0, z: 0, },
              })
              topic.publish(message)
      },
      onPanStart: function(e){
        this.txtConsole= "onPanStart: " + e.center.x+ ","+e.center.y;
        console.log(e);
      },
      onPanEnd: function(e){
        this.txtConsole= "onPanEnd: " + e.center.x+ ","+e.center.y;
        console.log(e);
      },
      onPress: function(e){
        this.txtConsole= "onPress: " + e.center.x+ ","+e.center.y;
        console.log(e);
      },
      onPressup: function(e){
        this.txtConsole= "onPressup:: " + e.center.x+ ","+e.center.y;
        console.log(e);
      }
    }
  }
</script>
<style>
 .box {
    touch-action: none;
    cursor: grab;
    width: 300px;
    height: 650px;
    
    border-radius: 1%;
  }
</style>