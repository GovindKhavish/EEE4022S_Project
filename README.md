# NMPC for Wheeled Robot Control
This repository hosts code and resources related to Nonlinear Model Predictive Control (NMPC) for wheeled robot navigation, with a strong emphasis on integrating it with ROS 2, Nav2, and Python programming. Our primary goal is to provide a comprehensive solution for controlling a wheeled robot using NMPC, leveraging the capabilities of ROS 2 and the Navigation2 stack.

## Description

The codebase presented here encompasses the implementation and testing of control algorithms tailored to wheeled robots. The central focus is on Nonlinear Model Predictive Control (NMPC), also ultizes ROS 2 and Navigation2 (Nav2). This powerful combination allows for precise control of a wheeled robot's trajectory within a dynamic environment.

The NMPC controller is designed to utilize odometry data and sensor information, incorporating the capabilities of ROS 2 to compute and execute precise control actions. The key highlight is the integration of Nav2 for seamless navigation, enabling the robot to autonomously follow predefined trajectories while adapting to real-time changes in its surroundings.

This codebase is not limited to the NMPC controller alone; it offers extensive Python programs that enhance the functionality and interaction with ROS 2. Additionally, it builds upon Nav2's navigation capabilities, ensuring the robot can efficiently plan, localize, and avoid obstacles.

## Getting Started

### Dependencies

* Describe any prerequisites, libraries, OS version, etc., needed before installing program.
* Ubunutu 22.04LTS
* Nav2
* ROS2 Humble
* Python 3.10

### Installing

* 

### Executing program

* How to run the program
* Step-by-step bullets
```
code blocks for commands
```

## Help

Note, within the bringup file in the Turtlebot3_nvaigation2 file, the world can be changed to incorprate a custom Gazebo world

```

```

## Authors

ex. Khavish Govind

## Version History

* 0.2
    * Various bug fixes and optimizations
    * See [commit change]() or See [release history]()
* 0.1
    * Initial Upload

## License

This project is licensed under the MIT License - see the LICENSE.md file for details

## Acknowledgments

Inspiration, code snippets, etc.
* [awesome-readme](https://github.com/matiassingers/awesome-readme)
* [PurpleBooth](https://gist.github.com/PurpleBooth/109311bb0361f32d87a2)
* [dbader](https://github.com/dbader/readme-template)
* [zenorocha](https://gist.github.com/zenorocha/4526327)
* [fvcproductions](https://gist.github.com/fvcproductions/1bfc2d4aecb01a834b46)
