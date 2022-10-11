import Vue from 'vue'
import VueRouter from 'vue-router'
import Home from '../views/Home.vue'

Vue.use(VueRouter)

const routes = [
  {
    path: '/',
    name: 'Home',
    component: Home
  },
  {
    path: '/ConnectROS',
    name: 'ConnectROS',
    // route level code-splitting
    // this generates a separate chunk (about.[hash].js) for this route
    // which is lazy-loaded when the route is visited.
    component: () => import(/* webpackChunkName: "about" */ '../views/ConnectROS.vue')
  },
  {
    path: '/ExDrawing1',
    name: 'ExDrawing1',
    component: () => import( '../views/ExDrawing1.vue')
  },
  {
    path: '/ExSwipe1',
    name: 'ExSwipe1',
    component: () => import( '../views/ExSwipe1.vue')
  },
  {
    path: '/ExDragDrop1',
    name: 'ExDragDrop1',
    component: () => import( '../views/ExDragDrop1.vue')
  }
  ,
  {
    path: '/ExTouchScreen1',
    name: 'ExTouchScreen1',
    component: () => import( '../views/ExTouchScreen1.vue')
  }
  , 
  {
    path: '/ExTouchCircle1',
    name: 'ExTouchCircle1',
    component: () => import( '../views/ExTouchCircle1.vue')
  }
  

  
]

const router = new VueRouter({
  routes
})

export default router
