# Project Name: HURO Metrics / ROS Mobile Interface 

## Overview
This repository contains two main components:

#### VueJS Mobile Application: 
A mobile application developed in VueJS that provides a user-friendly interface to interact with the ROS system. This application allows users to send commands and view real-time robot metrics from their mobile devices.

#### C++ Class (HuroMetrics):
A C++ class designed to interact with ROS (Robot Operating System) to calculate and log various metrics related to robot trajectory planning and execution.



## Connecting VueJS App to ROS

To connect the VueJS app to ROS:

1. Ensure `rosbridge_suite` is running on your ROS server:

    ```bash
    roslaunch rosbridge_server rosbridge_websocket.launch
    ```

2. Configure the WebSocket connection in the VueJS app by modifying the `config.js` file:

    ```js
    export const ROSBRIDGE_SERVER_URL = 'ws://your-ros-server-ip:9090';
    ```

3. Restart the VueJS app and verify the connection.



## License
This project is licensed under the MIT License.
