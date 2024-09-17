ARG ROS_VERSION=jazzy

FROM ros:${ROS_VERSION}-ros-base

ENV WORKSPACE=/data/workspace

WORKDIR $WORKSPACE

# O3DE Variables
ARG O3DE_REPO=https://github.com/o3de/o3de.git
ARG O3DE_BRANCH=stabilization/2409
ARG O3DE_COMMIT=HEAD
ENV O3DE_ROOT=$WORKSPACE/o3de

# O3DE Extras Variables
ARG O3DE_EXTRAS_REPO=https://github.com/o3de/o3de-extras.git
ARG O3DE_EXTRAS_BRANCH=stabilization/2409
ARG O3DE_EXTRAS_COMMIT=HEAD
ENV O3DE_EXTRAS_ROOT=$WORKSPACE/o3de-extras

# Other dependencies
ARG LOFT_GEM_REPO=https://github.com/o3de/loft-arch-vis-sample.git
ARG LOFT_GEM_BRANCH=main
ENV LOFT_GEM_ROOT=$WORKSPACE/loft-arch-vis-sample

# Project Variables
ARG PROJECT_DEMO_REPO=https://github.com/RobotecAI/rai-rosbot-xl-demo.git
ARG PROJECT_DEMO_BRANCH=main
ARG PROJECT_DEMO_COMMIT=HEAD
ENV PROJECT_DEMO_ROOT=$WORKSPACE/RAIROSBotXL
ENV PROJECT_DEMO_PROJECT=$PROJECT_DEMO_ROOT/Project

ENV LANG=en_US.UTF-8

# Install the required ubuntu packages
RUN apt-get update \
  && apt-get install -y \
  binutils \
  clang \
  cmake \
  git \
  git-lfs \
  libglu1-mesa-dev \
  libxcb-xinerama0 \
  libfontconfig1-dev \
  libnvidia-gl-470 \
  libxcb-xkb-dev \
  libxkbcommon-x11-dev \
  libxkbcommon-dev \
  libxcb-xfixes0-dev \
  libxcb-xinput-dev \
  libxcb-xinput0 \
  libpcre2-16-0 \
  ninja-build \
  python3-pip \
  software-properties-common \
  ros-${ROS_DISTRO}-ackermann-msgs \
  ros-${ROS_DISTRO}-control-toolbox \
  ros-${ROS_DISTRO}-gazebo-msgs \
  ros-${ROS_DISTRO}-joy \
  ros-${ROS_DISTRO}-navigation2 \
  ros-${ROS_DISTRO}-rviz2 \
  ros-${ROS_DISTRO}-tf2-ros \
  ros-${ROS_DISTRO}-urdfdom \
  ros-${ROS_DISTRO}-vision-msgs \
  ros-${ROS_DISTRO}-cyclonedds \
  ros-${ROS_DISTRO}-rmw-cyclonedds-cpp \
  ros-${ROS_DISTRO}-slam-toolbox \
  ros-${ROS_DISTRO}-nav2-bringup \
  ros-${ROS_DISTRO}-pointcloud-to-laserscan \
  ros-${ROS_DISTRO}-teleop-twist-keyboard \
  ros-${ROS_DISTRO}-topic-tools \
  python3-colcon-common-extensions \
  && rm -rf /var/lib/apt/lists/*

ENV RMW_IMPLEMENTATION=rmw_cyclonedds_cpp

# Clone and register all repositories
RUN . /opt/ros/${ROS_DISTRO}/setup.sh \
  \
  # Clone o3de \
  \
  && git clone --single-branch -b $O3DE_BRANCH $O3DE_REPO $O3DE_ROOT \
  && git -C $O3DE_ROOT lfs install \
  && git -C $O3DE_ROOT lfs pull \
  && git -C $O3DE_ROOT reset --hard $O3DE_COMMIT \
  \
  ############################################################### \
  # Clone o3de-extras   \
  ############################################################### \
  && git clone --single-branch -b $O3DE_EXTRAS_BRANCH $O3DE_EXTRAS_REPO $O3DE_EXTRAS_ROOT \
  && git -C $O3DE_EXTRAS_ROOT lfs install \
  && git -C $O3DE_EXTRAS_ROOT lfs pull \
  && git -C $O3DE_EXTRAS_ROOT reset --hard $O3DE_EXTRAS_COMMIT \
  \
  ############################################################### \
  # Clone dependencies   \
  ############################################################### \
  && git clone --single-branch -b $LOFT_GEM_BRANCH $LOFT_GEM_REPO $LOFT_GEM_ROOT \
  && git -C $LOFT_GEM_ROOT lfs install \
  && git -C $LOFT_GEM_ROOT lfs pull \
  \
  ############################################################### \
  # Clone the Project \
  ############################################################### \
  && git clone --single-branch -b $PROJECT_DEMO_BRANCH $PROJECT_DEMO_REPO $PROJECT_DEMO_ROOT \
  && git -C $PROJECT_DEMO_ROOT lfs install \
  && git -C $PROJECT_DEMO_ROOT lfs pull \
  && git -C $PROJECT_DEMO_ROOT reset --hard $PROJECT_DEMO_COMMIT \
  \
  ############################################################### \
  # Get the O3DE python and register the gem paths and projects \
  ############################################################### \
  && $O3DE_ROOT/python/get_python.sh \
  && $O3DE_ROOT/scripts/o3de.sh register -ep $O3DE_ROOT \
  && $O3DE_ROOT/scripts/o3de.sh register -gp $O3DE_EXTRAS_ROOT/Gems/ROS2 \
  && $O3DE_ROOT/scripts/o3de.sh register -gp $O3DE_EXTRAS_ROOT/Gems/RosRobotSample \
  && $O3DE_ROOT/scripts/o3de.sh register -gp $LOFT_GEM_ROOT/Gems/ArchVis

# Setup submodules
RUN cd $PROJECT_DEMO_ROOT && ./setup_submodules.bash

# Build the projects
RUN . /opt/ros/${ROS_DISTRO}/setup.sh \
  \
  ############################################################### \
  # Build the Editor, Tools, and clients
  ############################################################### \
  && cmake -B $PROJECT_DEMO_PROJECT/build/linux -G "Ninja Multi-Config" \
  -S $PROJECT_DEMO_PROJECT \
  -DLY_DISABLE_TEST_MODULES=ON \
  -DLY_STRIP_DEBUG_SYMBOLS=ON \
  && cmake --build $PROJECT_DEMO_PROJECT/build/linux --config profile \
  --target RAIROSBotXLDemo.GameLauncher Editor RAIROSBotXLDemo.Assets \
  \
  ############################################################### \
  # Remove build artifacts
  ############################################################### \
  && rm -rf ~/.o3de/3rdParty/ \
  && rm -rf $O3DE_ROOT/.git \
  && rm -rf $O3DE_ROOT/AutomatedTesting \
  && rm -rf $O3DE_ROOT/python/downloaded_packages \
  && rm -rf $PROJECT_DEMO_PROJECT/build/linux/Azcg/ \
  && rm -rf $PROJECT_DEMO_PROJECT/build/linux/CMake \
  && rm -rf $PROJECT_DEMO_PROJECT/build/linux/CMakeFiles/ \
  && rm -rf $PROJECT_DEMO_PROJECT/build/linux/External/ \
  && rm -rf $PROJECT_DEMO_PROJECT/build/linux/Testing/ \
  && rm -rf $PROJECT_DEMO_PROJECT/build/linux/_deps/ \
  && rm -rf $PROJECT_DEMO_PROJECT/build/linux/cmake \
  && rm -rf $PROJECT_DEMO_PROJECT/build/linux/lib/ \
  && rm -rf $PROJECT_DEMO_PROJECT/build/linux/o3de/ \
  && rm -rf $PROJECT_DEMO_PROJECT/build/linux/packages/

ENV NVIDIA_VISIBLE_DEVICES=all
ENV NVIDIA_DRIVER_CAPABILITIES=all
