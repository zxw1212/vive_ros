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

  Vive ROS package is made by two main nodes:

   -vive_ctrl: Publishes the hmd and controllers poses tracked by the system into a ROS topic in geometry_msgs/PoseStamped format. In addition, it publishes also the commands coming from the controller buttons
   
   -vive_hmd: Reads two topics in sensor_msgs/ImageCompressed format and stream the input image streams into the lenses of the hmd.


![-----------------------------------------------------](https://raw.githubusercontent.com/andreasbm/readme/master/assets/lines/rainbow.png)

<!-- REQUIREMENTS -->
<h2 id="requirements"> :warning: Requirements</h2>  
  
**Hardware Requirements:** 
- GPU based system (>= Scheda grafica NVIDIA GeForce RTX 2070 Max-Q)
  
**System Requirements:**
- OS: Ubuntu 18.04 lts
- Nvidia GPU Drivers : >=460.39
- Compiler: cmake C/CXX GNU 7.5.0
- Framework: Ros Melodic   
  
**Software requirements:**
- libglew-dev
- OpenVR : 1.3.22
- Steam : 1.0.0.68
- SteamVR : 1.16.8 
  
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

<!-- USAGE -->
<h2 id="usage"> :gear: Usage</h2>

![-----------------------------------------------------](https://raw.githubusercontent.com/andreasbm/readme/master/assets/lines/rainbow.png)

<h2 id="troubleshoot"> :gear: Troubleshoot</h2>

  A list of the most common problems found during installation and usage:

  - **SteamVR Fail: Error 307**

    SteamVR takes the wrong Vulkan implementation. Try this:
     ```sh
     sudo mv /usr/share/vulkan/icd.d/intel_icd.x86_64.json /usr/share/vulkan/icd.d/intel_icd.x86_64.json.disabled
     ```
