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
    <li><a href="#how-it-works"> ➤ How it works</a></li>
    <li>
      <a href="#usage"> ➤ Usage</a>
      <ul>
        <li><a href="#Sender-side">Sender side</a></li>
        <li><a href="#Receiver-side">Receiver side</a></li>
      </ul>
    </li>
    <li><a href="#troubleshoot"> ➤ Troubleshoot</a></li>
    <li><a href="#contributors"> ➤ Contributors</a></li>

  </ol>
</details>

![-----------------------------------------------------](https://raw.githubusercontent.com/andreasbm/readme/master/assets/lines/rainbow.png)

<!-- ABOUT THE PROJECT -->
<h2 id="about-the-project"> :pencil: About The Project</h2>

  Vive ROS package is made by two nodes:

   -vive_ctrl: Publishes the hmd and controllers poses tracked by the system into a ROS topic in geometry_msgs/PoseStamped format. In addition, it publishes also the commands coming from the controller buttons
   
   -vive_hmd: Reads two topics in sensor_msgs/ImageCompressed format and stream the input image streams into the lenses of the hmd.


![-----------------------------------------------------](https://raw.githubusercontent.com/andreasbm/readme/master/assets/lines/rainbow.png)

**Hardware requirements:**
- GPU based system (>= Scheda grafica NVIDIA GeForce RTX 2070 Max-Q)

**Software requirements:**:

- OS: Ubuntu 18.04 lts
- Nvidia GPU Drivers : >=460.39
- CUDA : >=11.2  
- Compiler: cmake C/CXX GNU 7.5.0
- Framework: Ros Melodic   
- Steam : 1.0.0.68
- SteamVR : 1.16.8 
- OpenVR : 1.3.22

![-----------------------------------------------------](https://raw.githubusercontent.com/andreasbm/readme/master/assets/lines/rainbow.png)

<!-- FOLDER STRUCTURE -->
<h2 id="folder-structure"> :cactus: Folder Structure</h2>

    vive_ros
    . 
    |
    ├── conf
    │   ├── pilot_index.yaml
    │   ├── pilot_vive.yaml
    │   └── teleop_bimanual.yaml
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
    │   ├── teleop_simulation.launch
    │   ├── vive_ctrl.launch
    │   ├── vive_framework.launch
    │   ├── vive_gazebo.launch
    │   ├── vive_hmd.launch
    |
    ├── rviz
    │   └── bimanual.rviz
    |
    ├── src
    │   ├── calib_sim.cpp
    │   ├── vive_ctrl.cpp
    │   ├── vive_hmd.cpp
    │   ├── vive_sim_ctrl.cpp
    |   └── vr_interface.cpp
    |
    ├── LICENSE
    ├── 60-HTC-Vive-perms.rules
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
  
- ### Install ROS package:
    1. #### Install additional ROS libraries
       ```sh 
       $ sudo apt-get install ros-melodic-tf -y
       $ sudo apt-get install ros-melodic-tf2* -y
       ``` 
    2. #### Install `vive_ros` package in the catkin_ws
       ```sh 
       $ cd ~/catkin_ws/src
       $ git clone https://github.com/JOiiNT-LAB/vive_ros.git
       $ cd ..
       $ catkin_make
       ```
    3. #### Add rules for the Vive devices:
       ```sh 
       $ cd ~/catkin_ws/src/vive_ros
       $ sudo cp ./60-HTC-Vive-perms.rules /etc/udev/rules.d
       $ sudo udevadm https://github.com/JOiiNT-LAB/vcontrol --reload-rules && sudo udevadm trigger
       ```               
- ### Steam and SteamVR installation:
    1. #### Download [Steam](https://store.steampowered.com) latest version. You should get the file steam_latest.deb in your ~/Downloads folder
    2. #### Run Steam 
       ```sh
       $ sudo dpkg --install ~/Downloads/steam_latest.deb
       $ steam
       ```
    3. #### Setup or log in into your Steam account and install SteamVR from the Steam store

![-----------------------------------------------------](https://raw.githubusercontent.com/andreasbm/readme/master/assets/lines/rainbow.png)

<!-- HOW IT WORKS -->
<h2 id="how-it-works"> :gear: How it works</h2>

- ### Settings:

   1.  The package can stream the positions and /tf for HTC controllers, HMD and lighthouses. In addition,
      it is possible to stream inside the HMD the images coming from a stereo-camera (Virtual and real).
      It is necessary to set the desired camera topics name in ```~/catkin_ws/src/vive_ros/launch/vive.launch``` as following:

         ```sh
         <arg name="image_left" default="/your_left_image_topic_name" />
         <arg name="image_right" default="/your_right_image_topic_name"/>
         ```

   2. Set `/vive/world_offset` and `/vive/world_yaw` to change the pose of the `world_vive` reference frame.
      
      ```sh
      <rosparam param="/vive/world_offset">[0, 0, 0]</rosparam>
      <rosparam param="/vive/world_yaw">0.0</rosparam>
      ```

- ### Steam and HTC Vive setup:

   1. Plug-in HTC Vive Base Stations
      
      a) Single base mode: select mode "a" with the button behind the base station

      b) Double base mode: select mode "b" on the first base and "c" on the second one, ensure the space between the two base stations
         is free.

   2. Plug-in and turn on the HTC Vive HMD

   3. Run the following commands in the terminal to give read/write permission for HTC Vive 
      ```sh
      $ sudo chmod a+rw /dev/hidraw*
      ```
   4. Start Steam and SteamVR:
      
      Open a new terminal and run
      
      ```sh
      $ steam
      ```
      
   5. Start SteamVR from Steam client

   6. Check if the HMD is inside the point of view of the stations and it is tracked by the stations

   7. Pair the HTC Joypads, check if they are tracked by the stations

- ### Launch ROS nodes:

   1. Start roscore:
      Open a new terminal and run:
      
      ```sh
      $ roscore
      ```
   2. Launch HMD and controllers nodes:
      
      Open a new terminal and run:
      
      ```sh
      $ roslaunch vive_ros vive.launch
      ```
   
   4. The sensor frame orientations are shown in the images below.
      
      (DISCLAIMER) The origin of the reference frames drawn is not the real one.

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

   Now launch Steam and SteamVR and then the nodes as already explained.

![-----------------------------------------------------](https://raw.githubusercontent.com/andreasbm/readme/master/assets/lines/rainbow.png)

# Troubleshoot:

## SteamVR Fail: Error 307

- SteamVR takes the wrong Vulkan implementation
Try this:
   ```sh
   sudo mv /usr/share/vulkan/icd.d/intel_icd.x86_64.json /usr/share/vulkan/icd.d/intel_icd.x86_64.json.disabled
   ```
