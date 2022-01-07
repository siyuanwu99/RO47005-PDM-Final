
<!-- PROJECT SHIELDS -->
<!--
*** I'm using markdown "reference style" links for readability.
*** Reference links are enclosed in brackets [ ] instead of parentheses ( ).
*** See the bottom of this document for the declaration of the reference variables
*** for contributors-url, forks-url, etc. This is an optional, concise syntax you may use.
*** https://www.markdownguide.org/basic-syntax/#reference-style-links
-->
[![Contributors][contributors-shield]][contributors-url]
[![Forks][forks-shield]][forks-url]
[![Stargazers][stars-shield]][stars-url]
[![Issues][issues-shield]][issues-url]
[![MIT License][license-shield]][license-url]
[![LinkedIn][linkedin-shield]][linkedin-url]


# Simulator

This simulator is based on the simulator used in [Fast-Planner](https://github.com/HKUST-Aerial-Robotics/Fast-Planner). It is a lightweight simulator in ROS considering quadrotor's dynamics in SO(3). This simulator has following features:

- SO3 dynamics
- local sensing
- randomly generated 3D map
-

# Structure

```
├── ./planner
│   ├── ./planner/example
│   └── ./planner/traj_utils
└── ./uav_simulator
    ├── ./uav_simulator/fake_drone
    ├── ./uav_simulator/local_sensing
    ├── ./uav_simulator/map_generator
    ├── ./uav_simulator/mockamap
    ├── ./uav_simulator/so3_control
    ├── ./uav_simulator/so3_quadrotor_simulator
    └── ./uav_simulator/Utils
        ├── ./uav_simulator/Utils/cmake_utils
        ├── ./uav_simulator/Utils/multi_map_server
        ├── ./uav_simulator/Utils/odom_visualization
        ├── ./uav_simulator/Utils/pose_utils
        ├── ./uav_simulator/Utils/quadrotor_msgs
        ├── ./uav_simulator/Utils/rviz_plugins
        ├── ./uav_simulator/Utils/uav_utils
        └── ./uav_simulator/Utils/waypoint_generator
```

## Installation

```shell
git clone `<this-repo>`
cd `<this-repo>`
catkin_make
```

## Run a simple demo
PRM:

```
roslaunch so3_quadrotor_simulator simulation_with_map.launch
```
RRT:

```
roslaunch rrt_planner simulation_with_map.launch

```


<!-- MARKDOWN LINKS & IMAGES -->
<!-- https://www.markdownguide.org/basic-syntax/#reference-style-links -->
[contributors-shield]: https://img.shields.io/github/contributors/edmundwsy/RO47005-PDM-Final.svg?style=for-the-badge
[contributors-url]: https://github.com/edmundwsy/RO47005-PDM-Final/graphs/contributors
[forks-shield]: https://img.shields.io/github/forks/edmundwsy/RO47005-PDM-Final.svg?style=for-the-badge
[forks-url]: https://github.com/edmundwsy/RO47005-PDM-Final/network/members
[stars-shield]: https://img.shields.io/github/stars/edmundwsy/RO47005-PDM-Final.svg?style=for-the-badge
[stars-url]: https://github.com/edmundwsy/RO47005-PDM-Final/stargazers
[issues-shield]: https://img.shields.io/github/issues/edmundwsy/RO47005-PDM-Final.svg?style=for-the-badge
[issues-url]: https://github.com/edmundwsy/RO47005-PDM-Final/issues
[license-shield]: https://img.shields.io/github/license/edmundwsy/RO47005-PDM-Final.svg?style=for-the-badge
[license-url]: https://github.com/edmundwsy/RO47005-PDM-Final/blob/master/LICENSE.txt
[linkedin-shield]: https://img.shields.io/badge/-LinkedIn-black.svg?style=for-the-badge&logo=linkedin&colorB=555
[linkedin-url]: https://linkedin.com/in/edmundwsy
[product-screenshot]: images/screenshot.png
