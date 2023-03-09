# semanticeight-ros

This repository contains the ROS interface of the object-level mapping and
exploration framework presented in the ICRA 2023 paper “Finding Things in the
Unknown: Semantic Object-Centric Exploration with an MAV”. The main repository
is [here](https://github.com/smartroboticslab/semantic-exploration-icra-2023).


## Dependencies

* Ubuntu 20.04 with ROS noetic
* GCC 7+ or clang 6+ (for C++ 17 features and OpenMP)
* CMake 3.10+
* Eigen 3
* OpenCV 3+
* yaml-cpp 5.2+

Install
the `ros-noetic-desktop-full` package by following the
[ROS Noetic installation instructions](http://wiki.ros.org/noetic/Installation/Ubuntu)
and then install the other dependencies:

``` sh
sudo apt-get --yes install g++ cmake libeigen3-dev libopencv-dev libyaml-cpp-dev python3-catkin-tools
```


## Build

``` sh
# Create and initialize a new catkin workspace.
source /opt/ros/noetic/setup.bash
mkdir -p ~/semanticeight_ws/src
cd ~/semanticeight_ws
catkin init

# Clone the required repositories.
cd ~/semanticeight_ws/src
git clone --recurse-submodules https://github.com/smartroboticslab/semanticeight-ros.git
git clone --recurse-submodules https://github.com/ethz-asl/mav_comm.git
# Skip the repository below if you plan on using the Habitat-based MAV simulator
# from here: https://github.com/smartroboticslab/habitat-mav-sim.git
git clone --recurse-submodules https://github.com/smartroboticslab/mav_interface_msgs.git

# Build the ROS package.
catkin build -DCMAKE_BUILD_TYPE=Release semanticeight_ros
source ~/semanticeight_ws/devel/setup.bash
```


## Usage

To get started have a look at [`launch/habitat.launch`](launch/habitat.launch)
and its respective config file, [`config/habitat.yaml`](config/habitat.yaml).
This launch file uses the Habitat-based MAV simulator from
[here](https://github.com/smartroboticslab/habitat-mav-sim.git).

Two MAV control interfaces are supported by semanticeight-ros, the
[SRL interface](https://github.com/smartroboticslab/mav_interface_msgs.git) and
the [RotorS interface](https://github.com/ethz-asl/mav_comm.git).


## License

Copyright 2019-2023 Smart Robotics Lab, Imperial College London, Technical University of Munich</br>
Copyright 2019 Anna Dai</br>
Copyright 2019-2023 Sotiris Papatheodorou</br>

semanticeight is distributed under the
[BSD 3-clause license](LICENSES/BSD-3-Clause.txt).
