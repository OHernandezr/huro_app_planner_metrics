import Vue from 'vue'
import App from './App.vue'
import vuetify from './plugins/vuetify'
import './registerServiceWorker'
import router from './router'
import VueSession from 'vue-session'

Vue.use(VueSession);
Vue.config.productionTip = false
Vue.prototype.$gTxtURLServer='ws://192.168.1.137:9090'

new Vue({
  vuetify,
  router, 
  render: h => h(App)
}).$mount('#app')
