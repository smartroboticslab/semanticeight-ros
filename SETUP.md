# supereight_ros setup guide

## 1. Install ROS Melodic

Follow the ROS Melodic setup guide from
[here](https://wiki.ros.org/melodic/Installation), depending on your OS. The
only officially supported OS is Ubuntu 18.04 (Bionic), since this is the only
OS where supereight_ros is tested. The rest of the instructions will assume you
are using ROS Melodic on Ubuntu 18.04.

After installing ROS, install the Python `catkin_tools` package

```bash
sudo apt install python-catkin-tools
```



## 2. Install supereight dependencies

supereight has the following dependencies:

- `CMake 3.10+`
- `Eigen 3`
- `Sophus 1.0.0`
- `OpenCV 3`

To install them run the provided script

```bash
./misc/install_supereight_dependencies.sh
```



## 3. Create a catkin workspace

### 3.1 Source ROS

In case you're not already doing so in your `.bashrc`

```bash
source /opt/ros/melodic/setup.bash
```

### 3.2 Create a new workspace and source directory

Feel free to change `~/catkin_ws` to a directory of your choice, making sure to
also change it in any following commands.

```bash
mkdir -p ~/catkin_ws/src
cd ~/catkin_ws
```

### 3.3 Initialize the workspace

```bash
catkin init
```



## 4. Clone and build supereight_ros

### 4.1 Clone supereight_ros into the workspace

```bash
cd ~/catkin_ws/src
git clone https://bitbucket.org/smartroboticslab/supereight_ros.git
```

### 4.2 Initialize and clone the supereight submodule

```bash
git submodule update --init
```

### 4.3 Build supereight and supereight_ros

```bash
catkin build -DCMAKE_BUILD_TYPE=Release
```

The above command will first build a compatible version of supereight and then
build supereight_ros.

