# Alpha AUV

## Installation

### MVP Installation

Currently MVP packages should be build from the source.
Target platform must be Ubuntu 20.04 because of the dependencies.

Pull repository and other dependencies
```bash
git clone https://github.com/GSO-soslab/mvp_msgs
git clone https://github.com/GSO-soslab/mvp_control
git clone https://github.com/GSO-soslab/mvp_mission
```

### ALPHA AUV Simulation and Hardware Installation

Pull repository and other dependencies
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

#### Install Stonefish Simulator
Pull the stonefish simulator library repository in somewhere other than ROS
workspace. Follow the installation instuctions at the Stonefish Readme.

```bash
git pull https://github.com/GSO-soslab/stonefish
```

Clone the Stonefish MVP at the ROS workspace.
```bash
git clone https://github.com/GSO-soslab/stonefish_mvp
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

### Test the installation
To run the simulation
```bash
roslaunch alpha_bringup bringup_simulation.launch
```

## Quick Start

Run the simulation and MVP Helm

```bash
roslaunch alpha_bringup bringup_simulation.launch
```

```bash
roslaunch alpha_bringup bringup_helm.launch
```

Start the local path tracking mission
```bash
rosservice call /helm/change_state "state: 'survey_local'"
```

## Funding
This work is supported by the [National Science Foundation](https://www.nsf.gov/) award [#2154901](https://www.nsf.gov/awardsearch/showAward?AWD_ID=2154901&HistoricalAwards=false)