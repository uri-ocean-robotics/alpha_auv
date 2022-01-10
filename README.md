# ALPHA AUV software stack


## Installation

Pull repository
```bash
git clone https://github.com/GSO-soslab/alpha_auv
cd alpha_auv
git submodule update --init --recursive
```
> Username password login via git command is not supported by github anymore. Use ssh keys or personal access tokens.

Install pip and setup python3 as default
```bash
sudo apt install python3-pip
```

Install dependencies
```bash
rosdep install --from-paths src --ignore-src --rosdistro ${ROS_DISTRO} -y
```

## Quick Start

To run the simulation
```bash
roslaunch alpha_bringup bringup_simulation.launch
```

To run the teleop node
```bash
roslaunch alpha_teleop simple_teleop.launch
```

To run controller node
```bash
roslaunch alpha_control alpha_control.launch
```
