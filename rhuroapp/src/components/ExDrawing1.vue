<template>
  <section>
    <div class="flex-row">
      <div class="source">
        <p>Canvas:</p>
        <vue-drawing-canvas
          ref="VueCanvasDrawing"
          :image.sync="image"
          :lineWidth="line"
          :background-color="backgroundColor"
          saveAs="jpeg"
          :styles="{
            'border': 'solid 1px #000'
          }"
          @mousemove="getCoordinate($event)"
          @mousepress="getCoordinate($event)"
        />
       <v-btn @click="coordenadas()"  >Coordenadas </v-btn>
      </div>
      <div class="output">
        <p>Output:</p>
        <img :src="image" style="border: solid 1px #000000">
      </div>
    </div>
  </section>
</template>
<script>
    import VueDrawingCanvas from "vue-drawing-canvas";

    export default {
      components: {
        VueDrawingCanvas,
      },
      data() {
        return {
          x: 0,
          y: 0,
          image: '',
          line: 5,
          color: '#000000',
          backgroundColor: '#FFFFFF',
          
        }
      },
  methods: {
    async setImage(event) {
      console.log('setImage');
      let URL = window.URL;
      this.backgroundImage = URL.createObjectURL(event.target.files[0]);
      await this.$refs.VueCanvasDrawing.redraw();
    },
    getCoordinate(event) {
      console.log('move');
      let coordinates = this.$refs.VueCanvasDrawing.getCoordinates(event);
      this.x = coordinates.x;
      this.y = coordinates.y;
    },
    coordenadas(event){
      let coordinates = this.$refs.VueCanvasDrawing.getCoordinates(event);
      console.log(coordinates);
    }
  }
};
</script>
<style scoped>
  .flex-row {
    display: flex;
    flex-direction: row;
  }
</style>
