# Autonomous Mobile Robot Demo

This repository is designed to demonstrate how the [RAI](https://github.com/RobotecAI/rai) framework interacts with the [ROSbot XL](https://husarion.com/manuals/rosbot-xl/), an autonomous mobile robot platform developed by [Husarion](https://husarion.com). The Husarion ROSBot XL provides a versatile and powerful platform for autonomous navigation, and this project showcases how RAI can be utilized to control and interface with the robot's sensors, actuators, and other onboard systems to achieve autonomous operation controlled by LLM.

The demo can be successfully run on any robot equipped with the same set of sensors and controlled using ROS 2 messaging protocols. Additionally, the same behavior can be reproduced in the simulation developed using [O3DE](https://www.o3de.org/) game engine. 

## The content

The following packages are available in this repository:
- `led_strip` ROS 2 package that allows **RAI** to use lights of the Husarion ROSBot XL robot
- `rosbot_xl_whoami` ROS 2 package that allows **RAI** to learn about the Husarion ROSBot XL robot and its features
- `RAIROSBotXLDemo` O3DE project for simulating Husarion ROSBot XL robot

`RAIROSBotXLDemo` is based on the [RobotVacuumSample demo](https://github.com/o3de/RobotVacuumSample) that was a part of the O3DE showcase at ROSCon conference in 2022. The O3DE demo is provided as a binary package for Ubuntu 24.04 with ROS 2 Jazzy and Ubuntu 22.04 with ROS 2 Humble, ensuring quick and easy setup. The binary package is the recommended method to run the demo. The source code and a Dockerfile are also available for those who prefer to build the project themselves or run it in a containerized environment. Please see the detailed demo [description](./docs/o3de.md) for more details.

## Demo description

The robot equipped with a microphone can listen to the commands and react accordingly. The demo is limited by the current Generative AI models linked via RAI framework and the robot's sensors. A simple proof-of-concept implementation shows, that RAI can interpret the commands from the operator and trigger actions on the robot. The robot can be either simulated or real thanks to ROS 2 middleware.

### Folder structure

- `docker` - _Dockerfile_ for O3DE project
- `docs` - files related to the documentation
- `Examples` - ROS 2 sample launchfiles for O3DE project
- `patches` - patches that are used to modify base O3DE project into `RAIROSBotXLDemo`
- `Project` - folder used to pull the base O3DE project as a _git submodule_ (please see [O3DE project description](./docs/o3de.md) for more details)
- `src` - ROS 2 source folder containing ROS 2 packages that can be used with real and simulated robots

## Screenshots

![Screenshot0](docs/images/husarion.png)

A sample setup with a Husarion ROSBot XL robot entering the room. 
Top left: color and depth images seen by the robot;
Top right: map constructed by the robot based on lidar data;
Bottom: photo of the robot entering the scene.

![Screenshot1](docs/images/o3deSimulation.png)

Simulation environment using O3DE.
