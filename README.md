# Autonomous Driving Project (AD17)

## System Requirements
- Ubuntu 20.04.6 LTS (64-bit)

## Tools Installation
Ensure you have the following tools installed on your system:

1. Basic Tools:
```bash
sudo apt install build-essential
sudo apt install git
sudo apt-get install terminator
```

2. ROS Installation:
Follow these steps to install ROS (Noetic version):

```bash
sudo sh -c 'echo "deb http://packages.ros.org/ros/ubuntu $(lsb_release -sc) main" > /etc/apt/sources.list.d/ros-latest.list'
sudo apt install curl
sudo apt update
curl -s https://raw.githubusercontent.com/ros/rosdistro/master/ros.asc | sudo apt-key add -
sudo apt install ros-noetic-desktop-full
sudo apt install python3-catkin-tools
```

3. Update .bashrc:
To ensure ROS is sourced correctly, update the .bashrc file:

```bash
echo "source /opt/ros/noetic/setup.bash" >> ~/.bashrc
```

## Packages Installation
Before proceeding, install the following ROS packages:

```bash
sudo apt-get install ros-noetic-nodelet
sudo apt-get install ros-noetic-depth-image-proc
sudo apt-get install ros-noetic-octomap
sudo apt-get install ros-noetic-octovis
sudo apt-get install ros-noetic-octomap-msgs
sudo apt-get install ros-noetic-octomap-rviz-*
sudo apt-get install ros-noetic-octomap-mapping
sudo apt-get install ros-noetic-octomap-server

sudo apt-get install ros-noetic-pcl-ros
sudo apt-get install ros-noetic-pcl-conversions
sudo apt-get install python3-pcl
sudo apt-get install python-opencv

sudo apt install ros-noetic-navigation
sudo apt install ros-noetic-actionlib
```

## Getting Started
Follow these steps to run the autonomous driving project:

1. Open a new terminal.
2. Clone the repository.
3. Build the project.

If any packages are missing, you can use `rosdep` to install them (be careful, this might causes problem with your current ROS installation):

```bash
cd autonomous_driving
sudo apt-get install python3-rosdep
sudo rosdep init
rosdep update
rosdep install --from-paths src --ignore-src -r -y
source /opt/ros/noetic/setup.bash
catkin build
source devel/setup.bash
```

Before running the system, ensure you have the simulation files ready:

1. Download the Unity Environment from: [Unity Environment Download Link](https://syncandshare.lrz.de/getlink/fiEg9ocZ6Pc5iuEa4QqN1b/)
2. Unzip the Unity file and copy the contents to `.../devel/lib/simulation/`
3. Make the simulation executable:
```bash
chmod +x ./devel/lib/simulation/Car_build.x86_64
```

Run the entire system:

```bash
source devel/setup.bash
roslaunch master.launch
```

Now, you should see both the Unity simulation window and the RViz visualization tool. RViz will open with a pre-configured file that allows you to visualize the entire autonomous driving pipeline.

## Code Editor (For Developers)
If you are a developer, you can use VS Code for coding:

1. Install VS Code:
```bash
sudo apt install code
```

2. Open the 'autonomous_driving' folder in VS Code.
3. In VS Code, go to the Extensions sidebar (Ctrl+Shift+X). Search for "ROS" in the search bar and choose the "ROS" extension developed by Microsoft.