############################################################
# Base Image for the Hardware Lab with UR3e + Robotiq Hand-E
############################################################
ARG ROS_DISTRO=jazzy
FROM osrf/ros:${ROS_DISTRO}-desktop

ENV ROS_DISTRO=${ROS_DISTRO}
ENV DEBIAN_FRONTEND=noninteractive
ENV RMW_IMPLEMENTATION=rmw_cyclonedds_cpp
ENV LANG=C.UTF-8
ENV LC_ALL=C.UTF-8
SHELL ["/bin/bash", "-c"]

# System utilities and core packages
RUN apt-get update && apt-get upgrade -y && apt-get install -y --no-install-recommends \
    autoconf automake bash-completion build-essential cmake make pkg-config g++ git curl wget unzip \
    ca-certificates software-properties-common python3-pip python3-setuptools python3-apt sudo \
    python3-rosdep python3-vcstool python3-colcon-common-extensions \
    lsb-release iproute2 iputils-ping net-tools socat \
    mc nano vim tree tmux tzdata screen less xclip figlet toilet \
    libxext-dev libx11-dev x11proto-gl-dev libgl1-mesa-dev libglu1-mesa-dev mesa-utils \
    libgtk-3-dev libxrandr-dev libxinerama-dev libxcursor-dev libxi-dev \
    libusb-1.0-0-dev libusb-1.0-0 libssl-dev libudev-dev udev libudev1 \
    python3-opencv \
    ros-${ROS_DISTRO}-cv-bridge \
    ros-${ROS_DISTRO}-image-transport \
    ros-${ROS_DISTRO}-tf2-ros \
    ros-${ROS_DISTRO}-tf2-geometry-msgs \
    ros-${ROS_DISTRO}-rmw-cyclonedds-cpp \
    ros-${ROS_DISTRO}-rviz2 \
    ros-${ROS_DISTRO}-librealsense2* \
    ros-${ROS_DISTRO}-realsense2-* \
    libpcl-dev libboost-dev libblas-dev liblapack-dev python3-dev \
    && rm -rf /var/lib/apt/lists/*

# Python packages
RUN pip install --no-cache-dir --break-system-packages \
    colcon-common-extensions vcstool ipython numpy==1.26.4 transforms3d==0.4.2 \
    opencv-python==4.9.0.80 opencv-contrib-python==4.9.0.80 \
    roboticstoolbox-python==1.1.1 python-fcl==0.7.0.8 \
    scipy tqdm matplotlib scikit-image scikit-learn spatialmath-python==1.0.5 \
    --extra-index-url https://pypi.org/simple && \
    pip install --no-cache-dir --index-url https://download.pytorch.org/whl/cpu torch torchvision ultralytics

# Librealsense installation
ARG LIBREALSENSE_TAG=v2.57.3
RUN git clone --depth 1 --branch ${LIBREALSENSE_TAG} https://github.com/IntelRealSense/librealsense.git /opt/librealsense \
 && cd /opt/librealsense && mkdir build && cd build \
 && cmake .. -DBUILD_EXAMPLES=true -DCMAKE_BUILD_TYPE=Release \
 && make -j"$(nproc)" && make install \
 && install -m 644 /opt/librealsense/config/99-realsense-libusb.rules /etc/udev/rules.d/99-realsense-libusb.rules || true

RUN python3 -m pip install pyrealsense2 --no-cache-dir --ignore-installed \
 || (cd /opt/librealsense && python3 -m pip install . --no-cache-dir --ignore-installed)

# Open3D from source
ARG OPEN3D_VERSION=v0.19.0
ENV OPEN3D_ML_ROOT=/opt/Open3D
WORKDIR /opt

RUN apt-get update && apt-get install -y --no-install-recommends \
    git cmake build-essential ninja-build python3-dev \
    libgl1-mesa-dev libegl1-mesa-dev libxi-dev libxrender-dev libxrandr-dev libxtst-dev \
    libgtk-3-dev libglfw3-dev libosmesa6-dev libudev-dev \
    libblas-dev liblapack-dev libjpeg-dev libpng-dev libtiff-dev \
    && rm -rf /var/lib/apt/lists/* && \
    git clone --branch ${OPEN3D_VERSION} --depth 1 https://github.com/isl-org/Open3D.git ${OPEN3D_ML_ROOT} && \
    cd ${OPEN3D_ML_ROOT} && mkdir build && cd build && \
    cmake .. \
      -DCMAKE_BUILD_TYPE=Release \
      -DBUILD_SHARED_LIBS=ON \
      -DBUILD_EXAMPLES=OFF \
      -DBUILD_GUI=OFF \
      -DBUILD_WEBRTC=OFF \
      -DBUILD_JUPYTER_EXTENSION=OFF \
      -DBUILD_TENSORFLOW_OPS=OFF \
      -DBUILD_PYTORCH_OPS=OFF \
      -DPYTHON_EXECUTABLE=$(which python3) \
      -DBUILD_UNIT_TESTS=OFF \
      -DBUNDLE_OPEN3D_ML=ON \
      -DCMAKE_INSTALL_PREFIX=/usr/local && \
    make -j"$(nproc)" && make install && \
    python3 -m pip install .

WORKDIR /home/robot

# MoveIt, UR Drivers, Gazebo, and Utilities
RUN apt-get update && apt-get install -y --no-install-recommends \
    ros-${ROS_DISTRO}-ros-gz \
    ros-${ROS_DISTRO}-ros2-control ros-${ROS_DISTRO}-ros2-controllers \
    ros-${ROS_DISTRO}-gz-ros2-control ros-${ROS_DISTRO}-ros-gz-bridge \
    ros-${ROS_DISTRO}-foxglove-bridge ros-${ROS_DISTRO}-joint-state-publisher-gui \
    ros-${ROS_DISTRO}-moveit* ros-${ROS_DISTRO}-ur* \
    ros-${ROS_DISTRO}-aruco-markers-msgs* ros-${ROS_DISTRO}-aruco-opencv* \
    ros-${ROS_DISTRO}-rqt-image-view ros-${ROS_DISTRO}-gz-* \
    && rm -rf /var/lib/apt/lists/*

# Gazebo tools
RUN apt-get update && apt-get install -y --no-install-recommends gnupg ca-certificates curl lsb-release && \
    curl -fsSL https://packages.osrfoundation.org/gazebo.gpg -o /usr/share/keyrings/osrf.gpg && \
    echo "deb [arch=$(dpkg --print-architecture) signed-by=/usr/share/keyrings/osrf.gpg] \
          http://packages.osrfoundation.org/gazebo/ubuntu-stable $(lsb_release -cs) main" \
          > /etc/apt/sources.list.d/gazebo-stable.list && \
    apt-get update && apt-get install -y libgz-tools2-dev && \
    rm -rf /var/lib/apt/lists/*

# Initialize and update rosdep
RUN sudo rosdep fix-permissions && \
    sudo rosdep init 2>/dev/null || true && \
    rosdep update

# UID/GID matching for host
ARG userid=1002
ARG groupid=1002
RUN existing_uid=$(getent passwd ${userid} | cut -d: -f1) && \
    if [ -n "$existing_uid" ]; then \
        echo "Reusing existing UID ${userid} (user: $existing_uid) as 'robot'"; \
        usermod -l robot -d /home/robot -m "$existing_uid"; \
    else \
        echo "Creating new user 'robot' with UID ${userid} and GID ${groupid}"; \
        groupadd -g ${groupid} -o robot || true; \
        useradd -ms /bin/bash --uid ${userid} -g ${groupid} --groups sudo,video robot; \
    fi && \
    echo '%sudo ALL=(ALL) NOPASSWD:ALL' >> /etc/sudoers

USER robot
ENV HOME=/home/robot
WORKDIR /home/robot
RUN sudo mkdir -p -m 0700 /run/user/${userid} && sudo chown ${userid}:${groupid} /run/user/${userid}

ENV USER=robot \
  LANG=en_US.UTF-8 \
  ROS_AUTOMATIC_DISCOVERY_RANGE=LOCALHOST \
  XDG_RUNTIME_DIR=/run/user/${userid} \
  LIBGL_ALWAYS_SOFTWARE=0 \
  TZ=America/New_York

# ROS 2 workspace
RUN source /opt/ros/${ROS_DISTRO}/setup.bash && \
    mkdir -p ~/ros2_ws/src && cd ~/ros2_ws/src && \
    git clone -b ${ROS_DISTRO} https://github.com/AGH-CEAI/robotiq_hande_driver.git && \
    vcs import . < robotiq_hande_driver/robotiq_hande_driver/robotiq_hande_driver.repos && \
    git clone -b ${ROS_DISTRO} https://github.com/UniversalRobots/Universal_Robots_ROS2_Driver.git ur_ros2_driver && \
    vcs import . < ur_ros2_driver/ur_ros2_driver.repos && \
    cd ~/ros2_ws && \
    colcon build --symlink-install --cmake-args -DCMAKE_BUILD_TYPE=Release

# code-server and VSCode ROS extension
RUN curl -fsSL https://code-server.dev/install.sh | sh && \
    if [ -f ./extensions/ms-iot.vscode-ros-0.9.6.vsix ]; then \
      code-server --install-extension ./extensions/ms-iot.vscode-ros-0.9.6.vsix; \
    fi || true

# Shell environment
RUN echo "source /opt/ros/${ROS_DISTRO}/setup.bash" >> ~/.bashrc && \
    echo "source ~/ros2_ws/install/setup.bash" >> ~/.bashrc && \
    echo "export RMW_IMPLEMENTATION=rmw_cyclonedds_cpp" >> ~/.bashrc && \
    echo "export QT_QPA_PLATFORM=xcb" >> ~/.bashrc && \
    echo "export GZ_VERSION=harmonic" >> ~/.bashrc && \
    echo "alias clc=clear" >> ~/.bashrc && \
    echo "clear" >> ~/.bashrc && \
    echo "toilet -f smblock --filter border:metal -w 24 ' ENEE 467   LAB 7  '" >> ~/.bashrc

# tmux Config
ADD --chown=robot:robot \
    https://raw.githubusercontent.com/MarylandRoboticsCenter/someConfigs/refs/heads/master/.tmux_K.conf \
    /home/robot/.tmux.conf

EXPOSE 11311
ENTRYPOINT ["/bin/bash", "-i"]
CMD ["rs-enumerate-devices", "--compact"]
