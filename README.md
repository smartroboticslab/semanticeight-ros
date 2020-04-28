<!-- SPDX-FileCopyrightText: 2019 Anna Dai -->
<!-- SPDX-FileCopyrightText: 2019-2020 Sotiris Papatheodorou -->
<!-- SPDX-License-Identifier: BSD-3-Clause -->

# supereight\_ros

A ROS wrapper for
[supereight](https://bitbucket.org/smartroboticslab/supereight-srl).



## Build

After cloning the repository, clone all the submodules using

``` bash
git submodule update --init
```

and then compile like any other ROS package. This will first build a compatible
version of supereight and then build supereight\_ros.



## Usage

There are a few `.launch` file for different use cases in `launch/`. Run the
simplest one using

``` bash
roslaunch supereight_ros supereight_ros.launch
```

You will most likely have to adjust the values of the `depth_image_topic`,
`rgb_image_topic`, `pose_topic` and `pose_topic_type` node arguments to suit
your current setup.

#### Subscribed topics

| Topic name            | Type                         | Description |
| :-------------------- | :--------------------------- | :---------- |
| `/camera/depth_image` | `sensor_msgs::Image`         | The input depth image. |
| `/camera/rgb_image`   | `sensor_msgs::Image`         | The input RGB image. Optional, only subscribed to if `enable_rgb` is `true`. |
| `/pose`               | `geometry_msgs::PoseStamped` | The external or ground truth camera pose. Its type can be changed to `geometry_msgs::TransformStamped` by changing the value of the `pose_topic_type` node argument. Optional, only subscribed to if `enable_tracking` is `false`. |

#### Published topics

| Topic name                  | Type                         | Description |
| :-------------------------- | :--------------------------- | :---------- |
| `/supereight/pose`          | `geometry_msgs::PoseStamped` | The camera pose as computed by supereight's ICP tracking. Only published if `enable_tracking` is `true`. |
| `/supereight/depth_render`  | `sensor_msgs::Image`         | The depth image received by supereight. Only published if `enable_rendering` is `true`. |
| `/supereight/rgb_render`    | `sensor_msgs::Image`         | The RGB image received by supereight. Only published if both `enable_rendering` and `enable_rgb` are `true`. |
| `/supereight/track_render`  | `sensor_msgs::Image`         | The ICP tracking status as an image. Only published if both `enable_rendering` and `enable_tracking` are `true`. |
| `/supereight/volume_render` | `sensor_msgs::Image`         | The map render from the current pose. Only published if `enable_rendering` is `true`. |

### `rviz` Visualization

Some `rviz` configuration files are provided in `config/`. Run one of the
following commands launch `rviz` and load one of the configuration files,
depending on your needs.

Visualize only depth images

``` bash
rviz -f config/supereight_ros.rviz
```

Visualize depth and RGB images

``` bash
rviz -f config/supereight_ros_rgb.rviz
```



### Documentation

To compile the documentation in HTML format in the `doc` directory run

``` bash
rosdoc_lite .
```


## License

Copyright © 2019 Anna Dai<br>
Copyright © 2019-2020 Sotiris Papatheodorou<br>
Distributed under the BSD 3-Clause license.

