rsp_pkgs
=========

### PX4 SITL
* Download source code
```
git clone git@github.com:MohitGupta007/PX4-Autopilot.git --recurse
git fetch --tags git@github.com:PX4/PX4-Autopilot.git
```

* Install Gazebo Garden
```
sudo wget https://packages.osrfoundation.org/gazebo.gpg -O /usr/share/keyrings/pkgs-osrf-archive-keyring.gpg
echo "deb [arch=$(dpkg --print-architecture) signed-by=/usr/share/keyrings/pkgs-osrf-archive-keyring.gpg] http://packages.osrfoundation.org/gazebo/ubuntu-stable $(lsb_release -cs) main" | sudo tee /etc/apt/sources.list.d/gazebo-stable.list > /dev/null
sudo apt-get update
sudo apt-get install gz-garden
```

* Install dependencies
```
bash ./PX4-Autopilot/Tools/setup/ubuntu.sh
```

* Run SITL
```
cd ~/PX4-Autopilot
make px4_sitl gz_x500_depth
```

### Install QGround Control
Install using instructions in the link https://docs.qgroundcontrol.com/master/en/getting_started/download_and_install.html

### ROS2 interface
* Create px4 ros2 workspace

```
mkdir px4_ws && cd px4_ws
mkdir src && cd src
```

* Download source code
```
git clone git@github.com:MohitGupta007/px4_pkgs.git
cd ~/px4_ws
colcon build
source install/setup.bash
ros2 run micro_ros_setup create_agent_ws.sh
ros2 run micro_ros_setup build_agent_ws.sh
cd ~/px4_ws
colcon build
```

* Install the uXRCE_DDS agent:
```
cd ~/px4_ws/src
git clone https://github.com/eProsima/Micro-XRCE-DDS-Agent.git
cd Micro-XRCE-DDS-Agent
mkdir build
cd build
cmake ..
make
sudo make install
sudo ldconfig /usr/local/lib/
```

* Get ros topics using
```
MicroXRCEAgent udp4 --port 8888
```
Incase of error, try the above command with sudo.

* Run the offboard example with aruco tags
In a new terminal run
```
cd ~/PX4-Autopilot
make px4_sitl gz_x500_depth
```
In the ignition gazebo window, you can add Image Display to view the camera image.

In another terminal run
```
ros2 launch rsp_quad rsp_quad.lauch
```