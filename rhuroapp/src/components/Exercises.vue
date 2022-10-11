<template>
  <v-container>
      <v-data-iterator :items="items"  hide-default-footer>
      <template v-slot:default="props">
        <v-row>
        
          <v-col v-for="item in props.items" :key="item.id_level" cols="12"  sm="6" md="4" lg="3">
           <v-card class="mx-auto"  max-width="300">
            <v-img
              :src="require('../assets/FisioterapiaSoma1.png')"
              class="my-3"
              contain
              height="200"
            />
              <v-card-title align="center" justify="center">
                {{ item.description }}
              </v-card-title>
              <v-card-subtitle align="left"  color="deep-purple accent-4">
                  Level {{ item.level }}  {{ item.level_description }}  <br/>
                  Daily Reps: 
                  <v-icon small color="green">{{ icons.mdiCheck }}</v-icon>
                  <v-icon small color="green">{{ icons.mdiCheck }}</v-icon>
                  <v-icon small color="green">{{ icons.mdiCheck }}</v-icon>
                  <v-icon small>{{ icons.mdiWindowClose }}</v-icon>
              </v-card-subtitle>
              
              <v-card-actions>
                <v-btn v-bind:to="item.route" color="orange lighten-2" text>Explore</v-btn>
                <v-spacer></v-spacer>
                <v-btn icon  @click="show = !show" >
                  <v-icon>{{ show ? 'mdi-chevron-up' : 'mdi-chevron-down' }}</v-icon>
                </v-btn>
              </v-card-actions>
              <v-expand-transition>
                <div v-show="show">
                  <v-divider></v-divider>
                  <v-card-text>
                  <b>{{ item.description }} Level {{ item.level }}  {{ item.level_description }}</b><br/>
                    ...
                  </v-card-text>
                </div>
              </v-expand-transition>
            </v-card>
          </v-col>
        </v-row>
      </template>
    </v-data-iterator>
  </v-container>
  
</template>

<script>
    import axios from 'axios'
    import { mdiCheck, mdiWindowClose, } from '@mdi/js'
    export default {
      name: 'Exercises',
      data: () => ({
          show: false,
          items: [
              {
                id:"1",
                description:"Touch the Circle",
                level:"1",
                route:"/ExTouchCircle1",
                level_description:"Basic",
                id_level:"1",
                image_url:"../assets/FisioterapiaSoma1.png"
              },
              {
                id:"2",
                "description":"Swipe",
                "level":"1",
                "route":"/ExSwipe1",
                "level_description":"Basic",
                "id_level":"4",
                "image_url":"img2.webp"}
                ],
          icons: {
              mdiCheck, 
              mdiWindowClose,
            },
      }),
      methods: { //http://10.10.51.106:8080/
          service1: function () {
            axios
              .get('http://192.168.1.136/rhuroapp_api_db/api.php/exercise/1')
              .then(response => (
                  this.items = response.data,
                  console.log(response.data)
                ))
               .catch(function (error) {
                 alert('Change the IP to connect to MySQL, detail: '+error.toJSON()+error)
              })
          }
    },
    beforeMount(){
        //this.service1()
    }
  }
</script>
