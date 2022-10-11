<template>
  <div id="app">
    <div class="box" ref="refEl" :style="transformStyle"></div>
  </div>
</template>
<script>
import { useDrag, useSpring } from 'vue-use-gesture'
import { defineComponent, computed, ref } from '@vue/composition-api'
export default defineComponent({
  setup() {
    //this.$emit('setConsole','hola')
    console.log('inicio')
    const refEl = ref()
    const transform = useSpring({ x: 0, y: 0 })
    // bind doesn't work for vue 2 bec of event names we can change listener names using vue-demi
    const bind = useDrag(
      ({ down, movement: [mx, my] }) => {
        console.log(mx)
        transform.x = down ? mx : 0
        transform.y = down ? my : 0
      },
      { domTarget: refEl }
    )
    console.log(bind())
    const transformStyle = computed(() => {
      const style = { transform: `translate3d(${transform.x}px,${transform.y}px,0)` }
      console.log("("+transform.x+","+transform.y+")")
      return style
    })
    return {
      refEl,
      bind,
      transformStyle,
    }
  },
})
</script>

<style>
.area {
  touch-action: none;
  cursor: -webkit-grab;
  cursor: grab;
}
 .box {
    touch-action: none;
    cursor: grab;
    width: 100px;
    height: 100px;
    background: blue;
    border-radius: 50%;
  }
</style>