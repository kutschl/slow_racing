# ss24_racing1

Provides an easy setup of a workspace for the f1tenth gym simulation in ROS 2 with the Dev Container extension of VS Code.

## Clone this repository

```
git clone --recursive https://gitlab.igg.uni-bonn.de/hrl_students/ss24_racing1.git
```

If you already cloned the repository without using --recursive, and want to check out the submodules:

```
git submodule init
git submodule update
```

## Prerequisites

You have to install VS Code, Docker, and the Dev Container extension for VS Code.
The installation of VS Code on Ubuntu is described [here](https://code.visualstudio.com/docs/setup/linux).
For a guide to set up dev containers, check out [this tutorial](https://code.visualstudio.com/docs/devcontainers/tutorial).
For installing docker on Ubuntu, you can use the docker apt repository, as described in the [docker docs](https://docs.docker.com/engine/install/ubuntu/).
The official docker packages of Ubuntu may also work, but may be older versions.

## NVIDIA GPU acceleration

To enable GPU acceleration in the containers, make sure you also install the [NVIDIA Container Toolkit](https://docs.nvidia.com/datacenter/cloud-native/container-toolkit/latest/install-guide.html). Note: If you already have the CUDA apt repository set up, it includes the nvidia-container-toolkit package, so you don't need to add the nvidia-container apt repository. Run:

```
sudo apt-get install -y nvidia-container-toolkit
sudo nvidia-ctk runtime configure --runtime=docker
sudo systemctl restart docker
```

In devcontainer.json, make sure "--gpus", "all" is added to runArgs (this is already the case for the workspaces in this repository), or configure it to use a specific device.

# Workspace setup

If you don't have a GPU , consider deleting the following line in runArgs from devcontainer.json: "--gpus", "all".

The workspace contains .devcontainer folder with the configuration files for the container.
To start up the container, you can press F1 and search for "Dev Containers: Open Folder in Container...", then select this folder.
Alternatively, you can navigate to the folder in a terminal and open it with "code .".
VS Code should recognize the .devcontainer configuration and ask you if you want to reopen the folder in the container.
If it doesn't, you can also press F1 and search for "Dev Containers: Rebuild and Reopen in Container".
If you change your configuration while being inside of the container and need to rebuild it, select "Dev Containers: Rebuild Container".

You may want to modify the files for your setup:

### devcontainer.json

The entry point for the devcontainer.
You may want to change the DOCKER_REPO, ROS_DISTRO and IMAGE_SUFFIX variables.
More information on which base image to choose can be found in the README of the [OSRF docker images repository](https://github.com/osrf/docker_images).
It also by default mounts the .ssh directory from your home folder into the docker home directory, so that you can use your keys from within.
This may not be necessary depending on your use case (devcontainers also have a [built-in functionality](https://code.visualstudio.com/remote/advancedcontainers/sharing-git-credentials) to share git credentials and ssh keys).

### Dockerfile

Pulls the docker image for the specified distribution.
Note that there are different versions of docker images with preinstalled packages available, so depending on your use case, you might want to switch to one of those (see devcontainer.json).
Also installs some basic packages.
You can add additional packages or remove the ones you don't need.
Workspace dependencies will be installed via rosdep when the container is started via the postCreateCommand, but these packages might have to be reinstalled if you change the workspace and have additional dependencies.
So it could be worth it to manually check your dependencies and add those here.
These will then be cached by Docker, so as long as you don't change the Dockerfile, you will not need to reinstall them.

### initialize.sh

This file is executed on the host machine during initialization.
Currently not used.

### postCreate.sh

This file is executed after the container has been created.
It installs missing workspace dependencies via rosdep and gives the container user ownership of the mounted workspace folder.
It also initializes and builds the workspace.

## Running the simulation

After the container is built, terminals you open in this VS code window should automatically open within the container, with the workspace sourced.
To run the simulation, use:

```
ros2 launch f1tenth_gym_ros gym_bridge_launch.py
```

You should see rviz opening, with the car and map visible.
To test controlling the robot, run:

```
ros2 run teleop_twist_keyboard teleop_twist_keyboard
```

in another terminal. Fore more informations about the simulation, see [f1tenth_gym_ros](./src/base_system/f1tenth_gym_ros) directory. 

## Running the real car 

After the container is built, terminals you open in this VS code window should automatically open within the container, with the workspace sourced.

```
ros2 launch f1tenth_stack bringup_launch.py
```

Fore more informations about the simulation, see [f1tenth_system](./src/base_system/f1tenth_system)  directory. 

## Tasks

Your task is to write a node that can automatically navigate a car through different racing tracks, first in simulation, then on the real cars.
You can decide yourself on the approach you want to take to accomplish that, but here are some sub-tasks that will probably be useful to accomplish:

- [X] Create a map of the racing track (e.g. with the [SLAM toolbox](https://github.com/SteveMacenski/slam_toolbox), see also [NAV2 tutorial](https://navigation.ros.org/tutorials/docs/navigation2_with_slam.html)).
- [X]  Localize the car within the map. This can also be done using the SLAM toolbox, or you can set up your own combination of [map server](https://navigation.ros.org/configuration/packages/configuring-map-server.html) and e.g. [AMCL](https://navigation.ros.org/configuration/packages/configuring-amcl.html) node.
- [X]  Generate a global path through the race track for the car to follow.
- [ ]  Write a reinforcement learning node that could use laser scan, map, location, velocities, and global path as input, and outputs linear/angular velocity commands for the car.
You can of course decide whether you want additional or less inputs.
You can also decide to write a node that doesn't use reinforcement learning, and instead relies on algorithms to navigate through the track.
- [ ] Instead of training in the ROS environment, you could also train your agent directly in the [Gym environment](https://github.com/f1tenth/f1tenth_gym).
This could speed up the training.
However, the goal is for the trained agent to run in the ROS environment (and later on the real cars), so you need to make sure that you get comparable inputs.

## Resources

- [F1tenth website](https://f1tenth.org/): Several tutorials and list of research papers
- [F1tenth github](https://github.com/f1tenth): Official repositories