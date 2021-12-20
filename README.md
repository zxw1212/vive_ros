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
  
- ### ROS Melodic installation:
    1. #### Setup your sources.list
       ```sh
       $ sudo sh -c 'echo "deb http://packages.ros.org/ros/ubuntu $(lsb_release -sc) main" > /etc/apt/sources.list.d/ros-latest.list'
       ```
    2. #### Set up your keys
       ```sh
       $ sudo apt-key adv --keyserver 'hkp://keyserver.ubuntu.com:80' --recv-key C1CF6E31E6BADE8868B172B4F42ED6FBAB17C654
       ```
       If you experience issues connecting to the keyserver, you can try substituting hkp://pgp.mit.edu:80 or hkp://keyserver.ubuntu.com:80 in the previous command.
       Alternatively, you can use curl instead of the apt-key command, which can be helpful if you are behind a proxy server: 
       ```sh
       $     curl -sSL 'http://keyserver.ubuntu.com/pks/lookup?op=get&search=0xC1CF6E31E6BADE8868B172B4F42ED6FBAB17C654' | sudo apt-key add -
       ```
    3. #### Install ROS Melodic Full version
       ```sh
       $ sudo apt update
       $ sudo apt install ros-melodic-desktop-full
       ```
    4. #### Environment Setup
       ```sh
       $ echo "source /opt/ros/melodic/setup.bash" >> ~/.bashrc
       $ source ~/.bashrc
       ```
    5. #### Dependencies for building packages
       ```sh
       $ sudo apt install python-rosdep python-rosinstall python-rosinstall-generator python-wstool build-essential
       ```
    6. #### Initialize Rosdep
       ```sh
       $ sudo apt install python-rosdep
       $ sudo rosdep init
       $ rosdep update
       ```
    7. #### Create a ROS Workspace
       ```sh
       $ mkdir -p ~/catkin_ws/src
       $ cd ~/catkin_ws/
       $ catkin_make 
       ```
    8. #### Local Environment Setup
       ```sh
       $ echo "source ~/catkin_ws/devel/setup.bash" >> ~/.bashrc
       $ source ~/.bashrc
       ```
    9. #### Install additional ROS libraries
       ```sh 
       $ sudo apt-get install ros-melodic-tf -y
       $ sudo apt-get install ros-melodic-tf2* -y
       ``` 
    9. #### Install `vive_ros` package in the catkin_ws
       ```sh 
       $ cd ~/catkin_ws/src
       $ git clone https://github.com/JOiiNT-LAB/vive_ros.git
       $ cd ..
       $ catkin_make
       ```
    9. #### Install `vive_ros` package in the catkin_ws
       ```sh 
       $ cd ~/catkin_ws/src/vive_ros
       $ sudo cp ./60-HTC-Vive-perms.rules /etc/udev/rules.d
       $ sudo udevadm control --reload-rules && sudo udevadm trigger
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

<li><a href="#troubleshoot"> ➤ Troubleshoot</a></li>
A list of the most common problems found during installation and usage:

  ## (PROBLEM) SteamVR Fail: Error 307

  - (SOLUTION) SteamVR takes the wrong Vulkan implementation
  Try this:
     ```sh
     sudo mv /usr/share/vulkan/icd.d/intel_icd.x86_64.json /usr/share/vulkan/icd.d/intel_icd.x86_64.json.disabled
     ```
