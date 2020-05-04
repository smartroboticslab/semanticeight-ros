# supereight_ros

A ROS wrapper for [supereight](https://github.com/emanuelev/supereight), a high
performance template octree library and a dense volumetric SLAM pipeline
implementation.



## Build

This package targets ROS Melodic on Ubuntu 18.04. Any other configuration is
not tested and not officially supported.

After cloning the repository, clone all the submodules using

``` bash
git submodule update --init
```

and then compile like any other ROS package.

For a detailed setup guide see [SETUP.md](SETUP.md).



## Usage

There are a few `.launch` files for different use cases in `launch/`. Run the
simplest one using

``` bash
roslaunch supereight_ros supereight_ros.launch
```

You will most likely have to adjust the values of the `depth_image_topic`,
`rgb_image_topic` and `pose_topic` node arguments to suit your current setup.
Also have a look at `config/config.yaml` which contains the node parameters.

For a detailed usage guide see [USAGE.md](USAGE.md).



## Documentation

To compile the documentation in HTML format and view it in you default browser
run

``` bash
rosdoc_lite .
xdg-open doc/html/index.html
```



## License

Copyright © 2019-2020 Smart Robotics Lab, Imperial College London

Copyright © 2019 Anna Dai

Copyright © 2019-2020 Sotiris Papatheodorou

Distributed under the BSD 3-Clause license.

