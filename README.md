# ENEE 467 Fall 2025: Robotics Project Laboratory
## Lab 7: Autonomous Manipulation with ROS 2 on the Real UR3e-Hand-E Robot

This repository contains a Docker container for Lab 7 (Autonomous Manipulation with ROS 2 on the Real UR3e-Hand-E Robot) as well as the necessary code templates for completing the hardware procedure and exercises. Due to the introduction of several changes to the UR hardware stack in ROS 2 Jazzy as well as the more pronounced RTDE-ModBUS issues, this repo provides packages developed in and for ROS 2 Humble, although a few might be compatible with newer ROS 2 distros.

## Overview

![ROS 2 Humble](https://img.shields.io/badge/ROS2-Humble-blue)

Autonomous robotic manipulation unites perception, planning, and control to enable robots sense, reason, and act independently towards the realization of a prescribed task. This lab brings these elements together in ROS 2, enabling the real UR3e-Hand-E robot to detect, grasp, and place an object. Students will explore how these subsystems interact in real time and how autonomy developed in simulation transfers to physical hardware.

## Lab Software

To avoid software conflicts and increase portability, all lab software will be packaged as a Docker container. Follow the instructions below to get started.

## Building the Container

First check to see if the image is prebuilt on the lab computer by running the following command
```
docker image ls
```
If you see the image named `lab-7-hw-image` in the list then you can **skip** the build process.

To build the Docker container, ensure that you have [Docker](https://www.docker.com/get-started/) installed and the Docker daemon running.
* Clone this repository and navigate to the `docker` folder
    ```
    cd ~/Labs
    git clone https://github.com/ENEE467-F2025/lab-7.git
    cd lab-7/docker
    ```
* Build the image with Docker compose
    ```
    userid=$(id -u) groupid=$(id -g) docker compose -f lab-7-hw-compose.yml build
    ```

## Starting the Container

The lab computers contain a prebuild image so you will not have to build the image.
* Clone this repo to get the lab-7 code if you haven't done so already
    ```
    cd ~/Labs
    git clone https://github.com/ENEE467-F2025/lab-7.git
    cd lab-7/docker
    ```
* Enable X11 forwarding
    ```
    xhost +local:root
    ```
* Run the Docker container
    ```
    userid=$(id -u) groupid=$(id -g) docker compose -f lab-7-hw-compose.yml run --rm lab-7-hw-docker
    ```
* Once inside the container, you should be greeted with the following prompt indicating that the container is running
    ```
    (lab-7) robot@docker-desktop:~$
    ```
* Edit the lab-7 Python (ROS 2) code  within the `lab-7/src` folder from a VS Code editor on the host machine. The repo directory `lab-7/src`  is mounted to the Docker container located at `/home/robot/ros2_ws/src` so all changes will be reflected **inside** the container.

## Attaching the Docker Container to VSCode
To enable type hints and IntelliSense, after starting the container, run the following command from a new terminal on the lab machine (host) to attach the running container to VSCode:
```bash
code --folder-uri vscode-remote://attached-container+$(printf "$(docker ps -q --filter ancestor=lab-7-hw-image)" | od -A n -t x1 | sed 's/ *//g' | tr -d '\n')/home/robot/ros2_ws/src
```
The command will launch VSCode on your host and automatically attach it to the running container. Once connected, you should see the folders from your container’s `src` directory in the VSCode workspace. Next, install the Python extension inside the container to enable type hints (make sure to select the option labeled `Install in Container: lab-7-hw-image`).

## Lab Instructions

Please follow the <a href="#" target="_blank">lab manual</a> closely. All instructions are contained inside the lab manual.
