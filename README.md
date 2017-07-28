# NRC
Repository for Natural Robot Control
(Based off of devel branch of cs225a git repository: https://github.com/tmigimatsu/cs225a-dist.git)

## Complete Installation Instructions

## NRC Automatic Installation Instructions (for Ubuntu and Mac only)
1. If you have a Mac, install Brew (https://brew.sh/). If you have Ubuntu, install git (sudo apt install git)

2. Clone this repository

   ```git clone https://github.com/The-Dawwctor/nrc.git nrc.git```

3. Download sai2-simulation.zip from AFS and extract its contents into nrc.git/sai2-simulation.

4. Run the install script inside nrc.git. This will take a few minutes.

   ```sh install.sh```

5. Build the nrc applications

   ```sh make.sh```

## NRC Manual Installation Instructions (for any Linux distro or Mac)
1. Install Brew for Mac (https://brew.sh/) or have Ubuntu installed or any other distro with the following packages. Also install git (sudo apt-get install git). Windows will have to dualboot ubuntu or install a VM.

2. Install Cmake

   Linux: ```sudo apt-get install cmake```
   
   Mac: ```brew install cmake```
   
3. Install Eigen

   Linux:```sudo apt-get install libeigen3-dev```
    
   Mac:```brew install eigen```
    
4. Install TinyXML
 
   Linux:```sudo apt-get install libtinyxml2-dev```
    
   Mac:```brew install tinyxml2```

5. Install JsonCPP

   Linux:```sudo apt-get install libjsoncpp-dev```
    
   Mac:```brew install jsoncpp```
   
6. Install HiRedis Client

   Linux:```sudo apt-get install libhiredis-dev```
    
   Mac:```brew install hiredis```
   
7. Install GLFW + Other things Chai3D Needs

   Linux:
   ```
   sudo apt-get install libglfw3-dev
   sudo apt-get install xorg-dev
   sudo apt-get install freeglut3-dev
   sudo apt-get install libasound2-dev
   sudo apt-get install libusb-1.0-0-dev
   ```
    
   Mac:
   ```
   brew install glfw3
   brew install libusb
   ```
   
8. Install Redis-Server

   Linux:```sudo apt-get install redis-server```
    
   Mac:```brew install redis```
   
9. Install Yaml-CPP

   Linux:```sudo apt-get install libyaml-cpp-dev```
    
   Mac:```brew install yaml```
   
10. Clone, Build, Install RBDL
    Download a copy of the repository as a zip from https://bitbucket.org/rbdl/rbdl
    Extract into a folder (should be called rbdl)
    ```
    cd rbdl
    mkdir build 
    cd build
    cmake -DRBDL_BUILD_ADDON_URDFREADER=On -DRBDL_USE_ROS_URDF_LIBRARY=OFF ..
    make -j4
    sudo make install
    ```
   
11. Install Chai

    Download and extract the multiplatform release from : http://www.chai3d.org/download/releases
    Cd into the extracted folder
 
    ```
    mkdir build
    cd build
    cmake ..
    make -j4
    ```
    
12. Extract the Zip of SAI2 Simulation Library given to you and cd into the extracted folder.
    ```
    mkdir build
    cd build
    cmake ..
    make -j4
    ```
13. Clone the SAI2 Common library and Build
    ```
    git clone https://github.com/manips-sai/sai2-common.git
    cd sai2-common
    mkdir build
    cd build
    cmake ..
    make -j4
    ```
    
14. Clone This Repo
    ```
    sh make.sh
    ```

## SEMPRE-robot Installation
    Follow the instructions at (https://github.com/The-Dawwctor/sempre-robot)

## SHRDLURN-robot Installation
    Follow the instructions at (https://github.com/The-Dawwctor/shrdlurn-robot)

## Post-Installation
1. To run the SEMPRE-robot server, while in the SEMPRE-robot git repository, run
    ```
    ant interactive
    ./interactive/run @mode=robot -server -interactive
    ```
    and keep it running in a separate terminal.

2. To run the SHRDLURN-robot client, while in the SHRDLURN-robot git repository, run
    ```
    yarn start
    ```
    to open a localhost website.

    Alternatively, type
    ```
    yarn build
    yarn deploy
    ```
    and go to the website (the-dawwctor.github.io/shrdlurn-robot/) to see the web client.

    The website can be changed through the instructions on the SHRDLURN-robot git repository.

3. Inside bin of the nrc git repository, you will find visualizer, simulator, and nrc
   ```
   ./nrc &
   ./visualizer &
   ```
   Note this opens up the applications in the background, use "jobs" to see currently running jobs and "fg" or "bg" to foreground next job or background next job
   
   In order to run the nrc program, have 3 separate terminals open and start, in this order:
   ```
   ./simulator resources/nrc/world.urdf resources/nrc/kuka_iiwa.urdf kuka_iiwa
   ./nrc resources/nrc/world.urdf resources/nrc/kuka_iiwa.urdf kuka_iiwa
   ./visualizer resources/nrc/world.urdf resources/nrc/kuka_iiwa.urdf kuka_iiwa
   ```
   
4. Read the source code of NRC inside src/nrc/NRC.cpp and the URDF file src/nrc/kuka_iiwa.urdf and src/nrc/world.urdf to understand what is happening inside the robot code and how the robot is described both kinematically and graphically.
