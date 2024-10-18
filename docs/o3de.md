# O3DE Demo Description

## Introduction

The demo is provided as a binary package for Ubuntu 24.04 with ROS 2 Jazzy and Ubuntu 22.04 with ROS 2 Humble, ensuring quick and easy setup. The binary package is the recommended method to run the demo. The source code and a Dockerfile are also available for those who prefer to build the project themselves or run it in a containerized environment.

The project was tested on Ubuntu 22.04 with ROS 2 Humble and Ubuntu 24.04 with ROS 2 Jazzy. 

> **Note:** This demo is **not supported on Windows!**

## Building using docker environment

The _Dockerfile_ defined in [docker folder](./docker) will prepare the appropriate ROS 2 distribution based environment and build the components necessary to run the demo project simulator through the O3DE engine.

### Requirements

* [Hardware requirements of o3de](https://www.o3de.org/docs/welcome-guide/requirements/)
* Ubuntu 22.04 (Jammy) with ROS 2 Humble or Ubuntu 24.04 (Noble) with ROS 2 Jazzy
* At least 60 GB of free disk space
* Docker installed and configured
  * **Note** It is recommended to have Docker installed correctly and in a secure manner so that the docker commands in this guide do not require elevated privileges (sudo) in order to run them. See [Docker Engine post-installation steps](https://docs.docker.com/engine/install/linux-postinstall/) for more details.
* [Nvidia container toolkit](https://docs.nvidia.com/datacenter/cloud-native/container-toolkit/install-guide.html#docker)

### Building the Full Docker Image

The `Dockerfile` supports defining which version of Ubuntu+ROS to base the docker container on, and by default will support Ubuntu 24.04 (Noble) with the ROS 2 Jazzy distribution. It will build a Docker image that will contain the Editor and the demo launcher. The file is located in [docker](./docker) folder. Use the following command to build the image:

```
docker build -t rai-rosbot -f Dockerfile .
```

The build process may take over two hours depending on the hardware resource and network connectivity of the machine used to build the image.

### Running the Docker Image

GPU acceleration is required for running O3DE correctly. For running docker with support for GPU please follow the documentation for [docker run](https://docs.docker.com/engine/reference/commandline/run/).

Another option is to install and use [rocker](https://github.com/osrf/rocker):

```
rocker --x11 --nvidia --network="bridge" rai-rosbot
```

The above command will log you into the docker terminal that has the code built. You will find scripts to run the simulation in `/data/workspace/RAIROSBotXL` folder. The O3DE Editor is located under the following path: `/data/workspace/RAIROSBotXL/Project/build/linux/bin/profile/Editor`.

## Building the demo from the source

The simulation environment from [RobotVacuumSample demo](https://github.com/o3de/RobotVacuumSample) is used in this demo. The primary difference lies in the default robot model available. Specifically, the original repository utilized a vacuum cleaner model as its default robot, whereas the current repository has been configured to utilize [Husarion ROSbot XL](https://husarion.com/manuals/rosbot-xl/). A script that pulls the base repository as a submodule and applies a corresponding _Git patch_ is available on the repository and is the only valid way to pull the _submodule_. 

> **_NOTE:_** The [Project](../Project) folder in this repository, which is a git submodule, will be set in a *dirty* git state due to changes applied by the patch.

Please follow the instructions below to build the project. The instructions are based on a common base folder: $DEMO_BASE (absolute path). Install [ROS 2 first](https://docs.ros.org/en/humble/Installation/Ubuntu-Install-Debians.html) and `git-lfs` package to pull the binary files. Install all `ROS 2` packages that are [required by O3DE](https://docs.o3de.org/docs/user-guide/interactivity/robotics/project-configuration/).

1. Install `git-lfs` package and pull the codebase.
```bash
sudo apt-get install git-lfs
cd $DEMO_BASE
git clone https://github.com/RobotecAI/rai-rosbot-xl-demo.git
cd $DEMO_BASE/rai-rosbot-xl-demo
./setup_submodules.bash
```

2. Clone O3DE and register the engine

```bash
cd $DEMO_BASE
git clone https://github.com/o3de/o3de.git -b main
cd $DEMO_BASE/o3de
git lfs install
git lfs pull
python/get_python.sh
scripts/o3de.sh register --this-engine
```

3. Clone and register the ROS2 Gem locally; register the RosRobotSample Gem locally

```bash
cd $DEMO_BASE
git clone https://github.com/o3de/o3de-extras.git -b main
$DEMO_BASE/o3de/scripts/o3de.sh register -gp $DEMO_BASE/o3de-extras/Gems/ROS2
$DEMO_BASE/o3de/scripts/o3de.sh register -gp $DEMO_BASE/o3de-extras/Gems/RosRobotSample
```

4. Clone and register the Loft Scene project locally

```bash
cd $DEMO_BASE
git clone https://github.com/o3de/loft-arch-vis-sample.git -b main
cd $DEMO_BASE/loft-arch-vis-sample
git lfs install
git lfs pull
$DEMO_BASE/o3de/scripts/o3de.sh register -gp $DEMO_BASE/loft-arch-vis-sample/Gems/ArchVis
```

## Building

You can prepare the simulation executable with:
- development build, or
- exporting the project.

**Development build** is tied with the local assets and is focused on development. It also includes the _Editor_, which can ue used to modify the scene before running the simulation.

**Exporting the project** creates a bundled and portable version of the simulation that can be moved between PCs.

## Development build

1. Build development build

```bash
cd $DEMO_BASE/rai-rosbot-xl-demo/Project
cmake -B build/linux -G "Ninja Multi-Config" -DLY_STRIP_DEBUG_SYMBOLS=TRUE -DLY_DISABLE_TEST_MODULES=ON
cmake --build build/linux --config profile --target RAIROSBotXLDemo.Assets RAIROSBotXLDemo.GameLauncher Editor
```

2. Run simulation

```bash
$DEMO_BASE/rai-rosbot-xl-demo/Project/build/linux/bin/profile/RAIROSBotXLDemo.GameLauncher -bg_ConnectToAssetProcessor=0
```

## Exporting project

1. Export the project using the o3de export tool
```bash
$DEMO_BASE/o3de/scripts/o3de.sh export-project -es ExportScripts/export_source_built_project.py --project-path $DEMO_BASE/rai-rosbot-xl-demo/Project -assets --fail-on-asset-errors -noserver -out $DEMO_BASE/rai-rosbot-xl-demo/release --build-tools --seedlist $DEMO_BASE/rai-rosbot-xl-demo/Project/AssetBundling/SeedLists/husarion.seed --no-unified-launcher
```

2. Run the simulation
```bash
$DEMO_BASE/rai-rosbot-xl-demo/release/RAIROSBotXLDemoGamePackage/RAIROSBotXLDemo.GameLauncher
```

### Running the simulation and navigation stack

Scripts for starting the simulation and navigation stack are added to this repository:
1. `run-game.bash`: a bash script starting the game (development build); or `run-game-exported.bash`: a bash script starting the game (exported project).
2. `run-nav.bash`: a bash script starting the navigation stack.
3. `run-rviz.bash`: a bash script starting RViz software to interface the navigation. 

First, start the simulation with `run-game.bash` or `run-game-exported.bash` (depending on your build type), and then start `run-nav.bash` and `run-rviz.bash` in separate shells and give the navigation goal in RViz window.
