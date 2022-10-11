<template>
  <v-app >
    <v-card >
    <v-app-bar color="teal lighten-3" dark >
        <v-app-bar-nav-icon @click="drawer = true"></v-app-bar-nav-icon>
        <v-toolbar-title>HURO App</v-toolbar-title>
        <v-spacer></v-spacer>
        <v-btn icon to="/">  <v-icon>mdi-home</v-icon></v-btn>
        <v-btn icon>  <v-icon>mdi-magnify</v-icon></v-btn>
        <v-menu
        left
        bottom
      >
       <template v-slot:activator="{ on, attrs }">
          <v-btn
            icon
            v-bind="attrs"
            v-on="on"
          >
            <v-icon>mdi-dots-vertical</v-icon>
          </v-btn>
        </template>

        <v-list>
          <v-list-item
          >
          <child-component ref="myChild" />
            <v-list-item-title>Option 1</v-list-item-title>
          </v-list-item>
        </v-list>
           </v-menu>
    </v-app-bar>
        <v-container style="min-height:600px;">
          <v-main>
              <router-view/>
          </v-main>
        </v-container>
    
    <v-navigation-drawer  v-model="drawer" absolute temporary >
      <v-list nav dense>
        <v-list-item-group v-model="group" active-class="deep-purple--text text--accent-4" >
         <v-list-item to="/">
            <v-list-item-icon>
              <v-icon>mdi-home</v-icon>
            </v-list-item-icon>
            <v-list-item-title>Home</v-list-item-title>
          </v-list-item>
          <v-list-item to="/ConnectROS">
            <v-list-item-icon>
              <v-icon>mdi-tools</v-icon>
            </v-list-item-icon>
            <v-list-item-title>Connect ROS</v-list-item-title>
          </v-list-item>
        </v-list-item-group>
      </v-list>
    </v-navigation-drawer> 
     <br/> 
  </v-card>  
 
   <v-footer padless>
    <v-col
      class="text-center"
      cols="12">
      {{ new Date().getFullYear() }} — <strong>HURO  </strong>
    </v-col>
  </v-footer>
  </v-app>

  
</template>

<script>
   
    export default {
      name: 'App',
      props: ['counter'] ,
      data: () => ({
          drawer: false,
          group: null,
          txtConsole: null,
      }),
    };
</script>
