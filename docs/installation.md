# ALPHA AUV software stack

## Installation

Pull repository and other dependencies
```bash
git clone https://github.com/GSO-soslab/alpha_auv
git clone https://github.com/GSO-soslab/mvp_msgs
git clone https://github.com/GSO-soslab/mvp_control
git clone https://github.com/GSO-soslab/mvp_mission
git clone https://github.com/GSO-soslab/stonefish_mvp
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

### Install Stonefish

Clone the repository in a directory that is not in the ros workspace.
```bash
git clone https://github.com/GSO-soslab/stonefish
```

Build the stonefish library and install it.

```bash
cd stonefish
mkdir build
cd build
cmake -DCMAKE_BUILD_TYPE=Release ..
make -j$(nproc)
sudo make install
```

## Check the installation



To run the simulation
```bash
roslaunch alpha_bringup bringup_simulation.launch
```
