# Seaplane Manual

---

## Table of Contents

### Part 1: Gazebo and ROS 2 Simulation

1. [Setting Up the ROS 2 Workspace](#1-setting-up-the-ros-2-workspace)
2. [Setting Up the World and Aircraft Model for Gazebo Simulation](#2-setting-up-the-world-and-aircraft-model-for-gazebo-simulation)
3. [Writing the Autonomy Node](#3-writing-the-autonomy-node)
4. [Installing MAVROS](#4-installing-mavros)
5. [Installing the ArduPilot Gazebo Plugin](#5-installing-the-ardupilot-gazebo-plugin)
6. [Installing ArduPilot](#6-installing-ardupilot)
7. [Running the Simulation](#7-running-the-simulation)

### Part 2: Crossflight Servo Control

1. [Raspberry Pi Setup](#1-raspberry-pi-setup)
2. [Installing Docker](#2-installing-docker)
3. [Directory Setup](#3-directory-setup)
4. [Build the Docker Image](#4-build-the-docker-image)
5. [Run the Docker Container](#5-run-the-docker-container)
6. [Installing MAVROS Inside the Container](#6-installing-mavros-inside-the-container)
7. [Running the Crossflight Node](#7-running-the-crossflight-node)
8. [Standalone PyMAVLink Alternative](#8-standalone-pymavlink-alternative)

## 1. Setting Up the ROS 2 Workspace

### 1.1 Install ROS 2

Follow the instructions under the **System Setup**, **Install ROS 2**, and **Setup Environment** sections at the link below to install and configure ROS 2 Jazzy:

[https://docs.ros.org/en/jazzy/Installation/Ubuntu-Install-Debs.html](https://docs.ros.org/en/jazzy/Installation/Ubuntu-Install-Debs.html)

Once installed, run the following command to permanently source ROS 2 in every new terminal session:

```bash
echo "source /opt/ros/jazzy/setup.bash" >> ~/.bashrc
```

### 1.2 Create the Workspace

Create a ROS 2 workspace directory (referred to as `seaplane_ws` throughout this manual). This folder, along with all necessary files, is included with this manual. It is recommended to create it in the home directory.

```bash
mkdir seaplane_ws
cd seaplane_ws
mkdir src
```

### 1.3 Create the ROS 2 Package

Navigate into the `src` directory and create a package named `auto_plane`:

```bash
cd src
ros2 pkg create --build-type ament_python auto_plane --dependencies rclpy sensor_msgs mavros_msgs
```

### 1.4 Create Additional Directories

Inside the `auto_plane` package directory (where `setup.py` is located), create two additional folders named `config` and `launch`:

```bash
mkdir config && mkdir launch
```

### 1.5 Modify `setup.py`

Open `setup.py` and make the following changes:

**Add these imports at the top of the file:**

```python
import os
from glob import glob
```

**Add the following two lines inside the `data_files` list, directly below the `package.xml` entry:**

```python
(os.path.join('share', package_name, 'config'), glob('config/*.yaml')),
(os.path.join('share', package_name, 'launch'), glob('launch/*.launch.py')),
```

### 1.6 Build the Workspace

Navigate back to the `seaplane_ws` root directory and build the workspace:

```bash
cd ~/seaplane_ws
colcon build
```

After a successful build, the `build`, `log`, and `install` directories will appear alongside `src`.

---

## 2. Setting Up the World and Aircraft Model for Gazebo Simulation

### 2.1 Install Gazebo

```bash
sudo apt-get install ros-${ROS_DISTRO}-ros-gz
```

### 2.2 Configure Resource Paths

The `models` and `worlds` folders required for the simulation are located inside the `gazebo` folder provided with this manual. Run the following command to add them to Gazebo's resource path. Replace `pathtomodelsfolder` and `pathtoworldsfolder` with the actual paths on your system:

```bash
echo "export GZ_SIM_RESOURCE_PATH=\$GZ_SIM_RESOURCE_PATH:pathtomodelsfolder:pathtoworldsfolder" >> ~/.bashrc
```

### 2.3 Camera Configuration

The aircraft model includes a forward-facing depth camera angled 10 degrees upward. This angle can be adjusted by editing the `model.sdf` file located at:

```
/gazebo/models/cessna_aircraft_ardupilot/model.sdf
```

### 2.4 Gazebo–ROS 2 Camera Bridge

A bridge is required to map Gazebo camera topics to ROS 2 so that depth and visual image data are accessible from ROS 2 topics. This is configured via a `.yaml` file placed inside the `config` folder of the `auto_plane` package.

The required `.yaml` file is included in the `config` folder provided with this manual. Copy it to the appropriate location.

### 2.5 Install the Depth Image to LaserScan Package

```bash
sudo apt update && sudo apt install ros-jazzy-depthimage-to-laserscan -y
```

---

## 3. Writing the Autonomy Node

### 3.1 Create the Node File

Inside the `auto_plane` package directory (the inner `auto_plane` folder containing `__init__.py`), create the main autonomy script:

```bash
touch waypoint_nav.py
```

The complete `waypoint_nav.py` file is provided with this manual. Copy and paste its contents into this file. It implements the waypoint navigation and obstacle avoidance logic and contains inline comments explaining the code.

### 3.2 Update `setup.py` Entry Points

Ensure the `entry_points` section in `setup.py` is updated as follows so that the node can be run with `ros2 run`:

```python
entry_points={
    'console_scripts': [
        'waypoint_nav = auto_plane.waypoint_nav:main',
    ],
},
```

This has already been updated in the `setup.py` file provided with the manual.

### 3.3 Source ROS 2

Run the following command to ensure ROS 2 is sourced permanently:

```bash
echo "source /opt/ros/jazzy/setup.bash" >> ~/.bashrc
```

### 3.4 Launch File

A launch file is required that performs three tasks simultaneously:

- Starts the `waypoint_nav` node
- Starts the `depthimage_to_laserscan` node
- Publishes a static transform from the camera frame to `base_link`

The launch file is included in the `launch` directory of the workspace provided with this manual.

---

## 4. Installing MAVROS

Run the following commands in sequence:

```bash
sudo apt update
sudo apt install ros-jazzy-mavros ros-jazzy-mavros-extras -y
sudo apt install -y geographiclib-tools libgeographiclib-dev
sudo apt install -y libasio-dev
wget https://raw.githubusercontent.com/mavlink/mavros/ros2/mavros/scripts/install_geographiclib_datasets.sh
chmod +x install_geographiclib_datasets.sh
./install_geographiclib_datasets.sh
```

---

## 5. Installing the ArduPilot Gazebo Plugin

### 5.1 Install Dependencies

```bash
sudo apt update
sudo apt install libgz-sim8-dev rapidjson-dev
sudo apt install libopencv-dev libgstreamer1.0-dev libgstreamer-plugins-base1.0-dev \
    gstreamer1.0-plugins-bad gstreamer1.0-libav gstreamer1.0-gl

export GZ_VERSION=harmonic

sudo bash -c 'wget https://raw.githubusercontent.com/osrf/osrf-rosdep/master/gz/00-gazebo.list \
    -O /etc/ros/rosdep/sources.list.d/00-gazebo.list'

rosdep update
rosdep resolve gz-harmonic

# Navigate to your ROS 2 workspace before running the next command
rosdep install --from-paths src --ignore-src -y
```

### 5.2 Clone and Build the Plugin

Clone the repository into your home directory:

```bash
git clone https://github.com/ArduPilot/ardupilot_gazebo
cd ardupilot_gazebo
mkdir build && cd build
cmake .. -DCMAKE_BUILD_TYPE=RelWithDebInfo
make -j4
```

### 5.3 Update Plugin Path Variables

Run the following commands to add the plugin and resource paths permanently. If the repository was cloned to a location other than the home directory, update the paths accordingly:

```bash
echo 'export GZ_SIM_SYSTEM_PLUGIN_PATH=$HOME/ardupilot_gazebo/build:${GZ_SIM_SYSTEM_PLUGIN_PATH}' >> ~/.bashrc
echo 'export GZ_SIM_RESOURCE_PATH=$HOME/ardupilot_gazebo/models:$HOME/ardupilot_gazebo/worlds:${GZ_SIM_RESOURCE_PATH}' >> ~/.bashrc
```

---

## 6. Installing ArduPilot

Clone the ArduPilot repository into your home directory and install all prerequisites:

```bash
git clone --recurse-submodules https://github.com/ArduPilot/ardupilot.git
cd ~/ardupilot/Tools/environment_install
chmod +x prereqs-ubuntu.sh && ./prereqs-ubuntu.sh
source ~/.profile
```

Configure and build the SITL (Software In The Loop) target for a fixed-wing aircraft:

```bash
cd ~/ardupilot
./waf configure --board sitl
./waf plane
```

---

## 7. Running the Simulation

### 7.1 Configure the Launch Script

A shell script named `sitl_seaplane.sh` is provided with this manual and is located inside the `seaplane_ws` folder. Open it with any text editor and update the paths as indicated by the comments within the file.

Make the script executable:

```bash
chmod +x sitl_seaplane.sh
```

This script launches multiple terminal windows and executes all required commands simultaneously, eliminating the need to start each component manually.

### 7.2 Start the Simulation

Open two separate terminals and run the following:

**Terminal 1:**
```bash
./sitl_seaplane.sh
```

**Terminal 2:**
```bash
ros2 launch auto_plane waypoint_nav_launch.launch.py
```

---

# Crossflight Servo Control

This section describes how to run the Crossflight servo control system on a Raspberry Pi 5 using Docker containers. Docker is used here to ensure the code runs consistently across different operating systems and hardware configurations.

## 1. Raspberry Pi Setup

The Raspberry Pi 5 is flashed with **Raspberry Pi OS "Trixie"** (based on Debian 13) and configured with the following credentials:

| Field    | Value             |
|----------|-------------------|
| Hostname | `csdraspberrypi`  |
| Username | `csdrpi`          |
| Password | `csdrpi123`       |

The device is configured to connect automatically to the **NITK-NET** Wi-Fi network on boot.

## 2. Installing Docker

Run the following commands from the home directory:

```bash
sudo apt update && sudo apt upgrade -y
curl -fsSL https://get.docker.com -o get-docker.sh
sudo sh get-docker.sh
rm get-docker.sh
sudo usermod -aG docker $USER
newgrp docker
sudo systemctl enable docker.service
sudo systemctl start docker.service
docker info   # Verify installation
```

## 3. Directory Setup

Create the following directory structure in the home directory:

```
~/seaplane_docker/
├── seaplane_ws/      ← Copy the provided seaplane_ws folder here
└── Dockerfile        ← Copy the provided Dockerfile here
```

> **Note:** Alternatively, copy the entire `seaplane_ws` folder provided with this manual directly into `seaplane_docker`.

## 4. Build the Docker Image

Navigate into `seaplane_docker` and build the image:

```bash
cd ~/seaplane_docker
docker build -t seaplane .
```

This creates a Docker image named `seaplane`. The Dockerfile is configured to include only the `crossflight_pkg` folder from within `seaplane_ws`, not the `auto_plane` package.

## 5. Run the Docker Container

Once the image is built, initialise a container named `crossflight_box`:

```bash
docker run -it --name crossflight_box \
  --net=host \
  --ipc=host \
  -v ~/seaplane_docker/seaplane_ws:/root/seaplane_ws \
  seaplane
```

The `seaplane_ws` folder inside the container is volume-mounted to the corresponding folder on the Raspberry Pi. This means any changes made to that folder inside the container are immediately reflected on the host system, and vice versa.

## 6. Installing MAVROS Inside the Container

Once inside the container, run the following commands to install MAVROS:

```bash
sudo apt update
sudo apt install ros-jazzy-mavros ros-jazzy-mavros-extras -y
sudo apt install -y geographiclib-tools libgeographiclib-dev
sudo apt install -y libasio-dev
wget https://raw.githubusercontent.com/mavlink/mavros/ros2/mavros/scripts/install_geographiclib_datasets.sh
ros2 run mavros install_geographiclib_datasets.sh
```

## 7. Running the Crossflight Node

Open two terminals on the Raspberry Pi.

**Terminal 1 — Start the container:**
```bash
docker start -ai crossflight_box
```

**Terminal 2 — Open a second shell inside the container:**
```bash
sudo docker exec -it crossflight_box bash
```

**In Terminal 1, start the MAVROS node:**
```bash
ros2 run mavros mavros_node --ros-args -p fcu_url:='serial:///dev/ttyACM0'
```

**In Terminal 2, build and run the Crossflight node:**
```bash
cd /root/seaplane_ws
colcon build
source install/setup.bash
ros2 run crossflight_pkg imu_data
```

The terminal will display the pitch angle reported by the Crossflight flight controller. The servos connected to Crossflight FCU pins 4 and 5 will respond by rotating when the pitch angle exceeds or falls below a defined threshold.

---

## 8. Standalone PyMAVLink Alternative

A standalone Python script named `imu_data_pymavlink.py` is included with this manual. It replicates the behaviour of the ROS 2 node without requiring a ROS 2 installation.

**Install dependencies:**

```bash
sudo apt update && sudo apt install python3-pip python3-setuptools python3-wheel -y
sudo apt install python3-dev python3-serial libxml2-dev libxslt-dev -y
pip3 install --upgrade pymavlink pyserial
sudo usermod -aG dialout $USER
newgrp dialout
chmod +x imu_data_pymavlink.py
```

**Run the script:**

```bash
python3 imu_servo_pymavlink.py
```
