<p align="center"> 
  <img src="images/jntlb_logo.png" alt="jntlb_logo">
</p>

<h1 align="center"> Vive ROS package extended, based on <a href="https://github.com/knorth55/vive_ros">knorth55/vive_ros</a><i> </h1>
<h3 align="center"> ROS Interface package for HTC Vive and Valve Index </h3>  

<!-- TABLE OF CONTENTS -->
<h2 id="table-of-contents"> :book: Table of Contents</h2>

<details open="open">
  <summary>Table of Contents</summary>
  <ol>
    <li><a href="#about-the-project"> ➤ About The Project</a></li>
    <li><a href="#requirements"> ➤ Requirements</a></li>
    <li><a href="#folder-structure"> ➤ Folder Structure</a></li>
    <li>
      <a href="#installation"> ➤ Installation</a>
      <ul>
        <li><a href="#glew-installation">Glew library installation</a></li>
        <li><a href="#openvr-installation">OpenVR Installation</a></li>
        <li><a href="#ros-installation">ROS Installation</a></li>
        <li><a href="#steam-steamvr-installation">Steam/SteamVR Installation</a></li>
      </ul>
    </li>
    <li><a href="#usage"> ➤ Usage</a></li>
    <li><a href="#troubleshoot"> ➤ Troubleshoot</a></li>
    <li><a href="#contributors"> ➤ Contributors</a></li>

  </ol>
</details>

![-----------------------------------------------------](https://raw.githubusercontent.com/andreasbm/readme/master/assets/lines/rainbow.png)

<!-- ABOUT THE PROJECT -->
<h2 id="about-the-project"> :pencil: About The Project</h2>

  Vive ROS package is made by two nodes:

   -vive_ctrl: Publishes the hmd and controllers poses tracked by the system into a ROS topic in geometry_msgs/PoseStamped format. In addition, it publishes also the commands coming from the controller buttons. Runs at 100Hz.
   
   -vive_hmd: Reads two topics in sensor_msgs/ImageCompressed format and stream the input image streams into the lenses of the hmd. Runs 45 Hz.


![-----------------------------------------------------](https://raw.githubusercontent.com/andreasbm/readme/master/assets/lines/rainbow.png)

<!-- REQUIREMENTS -->
<h2 id="requirements"> :warning: Requirements</h2>  
  
**Hardware Requirements:** 
- VR-Ready GPU (>= Scheda grafica NVIDIA GeForce RTX 2070 Max-Q in our case)
  
**System Requirements:**
- OS: Ubuntu 20.04 lts
- Nvidia GPU Drivers : >=470.57.02
- Compiler: cmake C/CXX GNU 7.5.0
- Framework: Ros Noetic   
  
**Software requirements:**
- libglew-dev
- OpenVR : 1.3.22
- Steam API: v020 | package version: 1647446817
- SteamVR : 1.22.6 Beta version 
  
![-----------------------------------------------------](https://raw.githubusercontent.com/andreasbm/readme/master/assets/lines/rainbow.png)

<!-- FOLDER STRUCTURE -->
<h2 id="folder-structure"> :cactus: Folder Structure</h2>

    vive_ros
    . 
    |
    ├── conf
    │   ├── index_param.yaml
    │   └── vive_param.yaml
    |
    ├── images
    │   ├── controller_frame.jpg
    │   ├── hmd_frame.jpg
    │   ├── jntlb_logo.png
    │   └── lighthouse_frame.jpg
    |
    ├── include
    │   ├── hellovr_opengl_main.h
    │   ├── hellovr_vulkan_main.h
    │   └── vr_interface.h
    |
    ├── launch
    │   ├── vive_ctrl.launch
    │   ├── vive_framework.launch
    │   └── vive_hmd.launch
    |
    ├── src
    │   ├── vive_ctrl.cpp
    │   ├── vive_hmd.cpp
    |   └── vr_interface.cpp
    |
    ├── CMakeLists.txt
    ├── LICENSE
    ├── package.xml
    └── README.md

![-----------------------------------------------------](https://raw.githubusercontent.com/andreasbm/readme/master/assets/lines/rainbow.png)

<!-- INSTALLATION -->
<h2 id="installation"> :woman_technologist: Installation</h2>

- ### `GLEW` OpenGL Extension Wrangler library installation:
       ```sh
       $ sudo apt update
       $ sudo apt install libglew-dev
       ```
       
- ### Valve's OpenVR SDK installation:

    1. #### Download Valve's OpenVR SDK
       ```bash
       $ cd ~
       $ mkdir libraries
       $ cd libraries
       $ git clone https://github.com/ValveSoftware/openvr.git -b v1.3.22 
       ```
    2. #### Build the SDK
       ```sh
       $ cd openvr
       $ mkdir build
       $ cd build
       $ cmake -DCMAKE_BUILD_TYPE=Release ../
       $ make
       ```

- ### Steam and SteamVR installation:
    1. #### Download [Steam](https://store.steampowered.com) latest version. You should get the file steam_latest.deb in your ~/Downloads folder
    2. #### Run Steam 
       ```sh
       $ sudo apt install steam
       ```
    3. #### Setup or log in into your Steam account and install SteamVR from the Steam store

- ### Install ROS package:
    1. #### Install `vive_ros` package in the catkin_ws
       ```sh 
       $ cd ~/catkin_ws/src
       $ git clone https://github.com/JOiiNT-LAB/vive_ros.git
       $ cd ..
       $ catkin_make
       ```

![-----------------------------------------------------](https://raw.githubusercontent.com/andreasbm/readme/master/assets/lines/rainbow.png)

<!-- USAGE -->
<h2 id="usage"> :gear: Usage</h2>

- ### Launch ROS nodes:

    1. #### Start roscore:
        ```sh
          $ roscore
        ```
    2. #### Edit the launch file to select the desired VR-set and the topic names for the input video
        ```sh
          <!-- VALVE INDEX SPECS -->
          <arg name="interpupillar_distance" default="200.0"/>
          <arg name="cam_f" default="720"/>

          <!-- HTC VIVE SPECS -->
          <!-- <arg name="interpupillar_distance" default="100.0"/> -->
          <!-- <arg name="cam_f" default="600"/> -->

          <arg name="image_left" default="/camera/left/image_raw/compressed"/>
          <arg name="image_right" default="/camera/right/image_raw/compressed"/>
          <arg name="image_left_info" default="/camera/left/camera_info" />
          <arg name="image_right_info" default="/camera/right/camera_info" />

        ```
    3. #### Launch controllers node:
        ```sh
          $ roslaunch vive_ros vive_ctrl.launch
        ```
    4. #### Launch HMD node:
        ```sh
          $ roslaunch vive_ros vive_hmd.launch
        ```

    5. #### There is also a launcher dedicated to the Joiint Lab teleoperation framework called vive_framework launcher. It works similarly to the other launch files.
    
    4. #### The sensor frame orientations are shown in the images below, they are similar for the INDEX set. (DISCLAIMER) The origin of the reference frames drawn is not the real one. 

    ![Alt text](images/lighthouse_frame.jpg?raw=true "lighthouse frame")
  
    ![Alt text](images/hmd_frame.jpg?raw=true "lighthouse frame")
  
    ![Alt text](images/controller_frame.jpg?raw=true "lighthouse frame")

- ### Controllers tracking without HMD:

It is possible to use track the controllers without connecting the HMD, but the controllers must be connected via USB.
To set this mode:

  1. Enable a null (simulated) headset editing 
      ```sh
        gedit ~/.steam/steam/steamapps/common/SteamVR/resources/settings/default.vrsettings
      ```

  2. Change the third line from ```"requireHmd" : true``` to ``` "requireHmd" : false ```

  3. Add ```"activateMultipleDrivers" : true```

  4. Add the line ```"forcedDriver": "null"``` beneath it.

  5. Open ```default.vrsettings```
      ```sh
        $ gedit ~/.steam/steam/steamapps/common/SteamVR/drivers/null/resources/settings/default.vrsettings
      ```

  6. Set ```"enable": true``` to enable it.

  7. Now launch Steam and SteamVR and then the nodes as already explained.

![-----------------------------------------------------](https://raw.githubusercontent.com/andreasbm/readme/master/assets/lines/rainbow.png)

<h2 id="troubleshoot"> :gear: Troubleshoot</h2>

  A list of the most common problems found during installation and usage:
  
  - **You are missing the following 32-bit libraries, and Steam may not run: libc.so.6**
    SteamVR takes the wrong Vulkan implementation. Try this:
     ```sh
     sudo mv /usr/share/vulkan/icd.d/intel_icd.x86_64.json /usr/share/vulkan/icd.d/intel_icd.x86_64.json.disabled
     ```
