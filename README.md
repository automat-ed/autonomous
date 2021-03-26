# StreetSmart by AutomatED

![Continuous Integration](https://github.com/automat-ed/autonomous/workflows/Continuous%20Integration/badge.svg)

## Introduction

StreetSmart by AutomatED is an autonomous gritting robot, designed to navigate footpaths in a park environment to prevent the formation of ice that forms during the winter months with the ultimate aim of improving their safety for vulnerable citizens.

This repository defines the [ROS 1 Noetic](https://wiki.ros.org/noetic) workspace that houses our entire autonomous software stack (as well as our [simulation](https://github.com/automat-ed/simulation) software as a [git submodule](https://git-scm.com/book/en/v2/Git-Tools-Submodules)) and other configuration files (e.g launch files or RViz config files).

## Prerequisites
To run this package you will need to have installed:
* [ROS 1 Noetic](https://wiki.ros.org/noetic/Installation/Ubuntu)
* [Webots (R2021a)](https://www.cyberbotics.com/doc/guide/installation-procedure#installation-on-linux)

Click the links above to go to their respective installation pages.

## Installation
The following instructions details how to install the pacakge and its dependencies. We assume you have installed the packages listed in the [Prerequisites section](#prerequisites) and that you have sourced your ROS installation (using `source /opt/ros/noetic/setup.bash` or similar).
1. Clone the repository (and its submodules)
    ```bash
    git clone --recurse-submodules git@github.com:automat-ed/autonomous.git
    ```
2. Install package dependencies using [`rosdep`](https://wiki.ros.org/rosdep)
    ```bash
    rosdep install --from-paths src --ignore-src -y
    ```
3. Build workspace (catkin_make will also work if you don't have [catkin_tools](https://catkin-tools.readthedocs.io/) installed)
    ```bash
    catkin build
    ```
4. Set `AUTOMATED_HOME` environment variable to the absolute path to the workspace. For example:
    ```bash
    echo "export AUTOMATED_HOME=$HOME/autonomous" >> ~/.bashrc
    ```
    Or if you are using zsh:
    ```bash
    echo "export AUTOMATED_HOME=$HOME/autonomous" >> ~/.zshrc
    ```
5. Set `WEBOTS_HOME` environment variable to the `webots` directory obtained from installing Webots (typicall `/usr/loca/webots` on Linux):
    ```bash
    echo "export WEBOTS_HOME=/usr/local/webots" >> ~/.bashrc
    ```
    Or if you are using zsh:
    ```bash
    echo "export WEBOTS_HOME=/usr/local/webots" >> ~/.zshrc
    ```
6. Add to your `.bashrc` file (or `.zshrc` if you use zsh):
    ```bash
    export LD_LIBRARY_PATH="${WEBOTS_HOME}/lib/controller:$PATH"
    ```
7. Source workspace
    ```bash
    source devel/setup.bash
    ```
    Or if you are using zsh:
  
    ```bash
    source devel/setup.zsh
    ```
## Running
Running this package simply involves launching the `full.launch` launch file:
```bash
roslaunch launch/full.launch
```

You can pass in multiple commandline statements to `full.launch`
* `mode`: use to specify the simulation mode to start Webots in; must be one of `{pause, realtime, run, fast}`
* `no-gui`: determines whether a minimal gui should be launched for Webots or not
* `world`: the name of the world file to launch in `src/simulation/worlds`
* `keyboard`: the type of keyboard controller to use
    * 0 uses the keyboard controller provided by the [`simulation`](https://github.com/automat-ed/simulation) package
    * 1 uses the C++ keyboard controller in the `controllers` package
    * 2 uses the Python keyboard controller in the `controlelrs` package
    * Anything else disables the ability to control the robot with your keyboard
* `rviz_config`: Launches RViz with the RViz config file in the `rviz` directory with the matching name (without the file extension). Leaving it an empty string will cause RViz not to launch.
* `portal`: A configuration parameter that gets passed onto the state machine. Determines whether the state machine should wait for a connection to the Portal or not.
* `launch_file`: The absolute path to the launch file to be run by the state machine when leaving the OFF state.
* `ground_truth`: A configuration parameter that gets passed to the launch file launched by the state machine. When combined with `navigation.launch`, this enables to use of ground truth localization.
* `map_file`: A configuration parameter taht gets passed to the launch file launched by the state machine. When combined with `navigation.launch`, it determines the specific config file to pass to `map_server` from within the `src/navigation/config` directory.
* `eval`: Whether or not to launch the `data_log.py` script which will log the ground truth, `ekf_local` and `ekf_global` position outputs once a message to `move_base_simple/goal` has been published.
