# ENEE 467 Fall 2025: Robotics Project Laboratory
## Lab 7: Autonomous Manipulation with ROS 2 on the Real UR3e-Hand-E Robot

This repository contains a Docker container for Lab 7 (Autonomous Manipulation with ROS 2 on the Real UR3e-Hand-E Robot) as well as the necessary code templates for completing the hardware procedure and exercises. Due to its better ROS 2 compliant simulation support, this repo provides packages developed for Gazebo Harmonic and targeting ROS 2 Jazzy. 

## Overview

![ROS 2 Jazzy](https://img.shields.io/badge/ROS2-Jazzy-orange)

Autonomous robotic manipulation unites perception, planning, and control to enable robots sense, reason, and act independently towards the realization of a prescribed task. This lab brings these elements together in ROS 2, enabling the real UR3e-Hand-E robot to detect, grasp, and place an object. Students will explore how these subsystems interact in real time and how autonomy developed in simulation transfers to physical hardware.

## Lab Software

To avoid software conflicts and increase portability, a Docker image containing the simulation software has already been built on the lab computers. You can start a container like so:

## Starting the Container (this will only work only on a lab machine)

Before beginning, ensure you are on a lab machine. The lab computers contain a prebuild image and the source files already so you will not have to build the image. 

* Start by enabling X11 forwarding:
    ```
    xhost +local:root
    ```
* Then change directory to the `~/Labs/lab-7-sim/` folder:
    ```
    cd ~/Labs/lab-7-sim/docker
    ```
* Start the sim container:
    ```
    userid=$(id -u) groupid=$(id -g) docker compose -f lab-7-sim-compose.yml run --rm lab-7-sim-docker
    ```
* Once inside the container, you should be greeted with the following prompt indicating that the container is running
    ```
    (lab-7) robot@docker-desktop:~$
    ```
* As you will not be editing any files for the simulator part, the repo directory is **NOT** mounted to the Docker container.

## Attaching the Docker Container to VSCode
To enable type hints and IntelliSense, after starting the container, run the following command from a new terminal on the lab machine (host) to attach the running container to VSCode:
```bash
code --folder-uri vscode-remote://attached-container+$(printf "$(docker ps -q --filter ancestor=lab-7-sim-image)" | od -A n -t x1 | sed 's/ *//g' | tr -d '\n')/home/robot/ros2_ws/src
```
The command will launch VSCode on your host and automatically attach it to the running container. Once connected, you should see the folders from your container’s `src` directory in the VSCode workspace. Next, install the Python extension inside the container to enable type hints (make sure to select the option labeled `Install in Container: lab-7-sim-image`).

## Lab Instructions

Please follow the [lab manual](Lab_7_Autonomous_Manipulation_on_the_Real_Robot.pdf) closely. All instructions are contained inside the lab manual.