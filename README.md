# Robust Nonlinear Quadrotor Position Controller ROS Package
----

## Prerequisites

* ROS Noetic
* Gazebo 11
* Git
* Pip3
* QGroundControl (Optional)

---

## 1. Clone this repository
    git clone https://github.com/Sherif-Sameh/quadrotor_pos_control.git

---

## 2. Install Ardupilot and MAVProxy
Based off of this tutorial: https://github.com/Intelligent-Quads/iq_tutorials/blob/master/docs/Installing_Ardupilot.md

### a. Clone Ardupilot
Note: Replace the version of ArduCopter with the latest stable version currently.

    cd ~
    git clone https://github.com/ArduPilot/ardupilot.git
    cd ardupilot
    git checkout Copter-4.4.2
    git submodule update --init --recursive

### b. Install dependencies
    sudo apt install python3-matplotlib python3-serial python3-wxgtk3.0 python3-wxtools python3-lxml python3-scipy python3-opencv ccache gawk python3-pip python3-pexpect

### c. Install MAVProxy
    sudo pip3 install future pymavlink MAVProxy

### d. Modify `.bashrc`
Open `~/.bashrc` with any text editor
Add these lines at the end of the `.bashrc` file

    export PATH=$PATH:$HOME/ardupilot/Tools/autotest
    export PATH=/usr/lib/ccache:$PATH

Save and exit

### e. Test installation
    cd ~/ardupilot/ArduCopter
    sim_vehicle.py -w

---

## 3. Install Gazebo APM Plugin
### a. Clone the repository and build from source
    cd ~
    git clone https://github.com/khancyr/ardupilot_gazebo
    cd ardupilot_gazebo
    mkdir build
    cd build
    cmake ..
    make -j4
    sudo make install

If not already done during Gazebo's installation:
    
    echo 'source /usr/share/gazebo/setup.sh' >> ~/.bashrc

### b. Modify default world and model description files
    cd /usr/share/gazebo-11/
    ls models/

If the `iris_with_ardupilot` and `iris_with_standoffs` directories exist:

    sudo rm -r /models/iris_with_*

Copy the modified directories:

    sudo cp -r ~/quadrotor_pos_control/utils/iris_with_* ./models/

Modify the world decription file:

    cd worlds/
    sudo rm iris_arducopter_runway.world
    sudo cp ~/quadrotor_pos_control/utils/iris_arducopter_runway.world ./

### c. Test the installation
Launch Ardupilot SITL in the 1st terminal:

    cd ~/ardupilot/ArduCopter
    sim_vehicle.py -v ArduCopter -f gazebo-iris

Launch Gazebo in the 2nd terminal:

    gazebo --verbose worlds/iris_arducopter_runway.world

---

## 4. Install additional dependencies

### a. Install `Eigen3`:
    sudo apt-get update
    sudo apt-get install libeigen3-dev

### b. Install `qpOASES`:

    cd ~
    git clone https://github.com/coin-or/qpOASES.git
    cd ~/qpOASES
    mkdir build && cd build
    cmake .. -DCMAKE_CXX_FLAGS=-fPIC
    sudo make install

Run a test program:

    cd ~/qpOASES/bin
    ./example1

If the program terminates successfully then installation was successful.

---

## 5. Initialize the ROS workspace and build from source
Install `catkin build` if not installed:

    sudo apt update
    sudo apt install python3-catkin-tools

Initialize the workspace:

    cd ~
    mkdir -p ~/catkin_ws/src
    cd ~/catkin_ws/src
    catkin_init_workspace
    cd ~/catkin_ws
    catkin build

Copy source code into the src folder:

    rm -r src/*
    cp -r ~/quadrotor_pos_control/src/* ./src/

Build from source:

    catkin build
    source devel/setup.bash

Test the workspace:
    
    roslaunch test_package system.launch