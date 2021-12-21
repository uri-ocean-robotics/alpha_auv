# ALPHA AUV software stack


## Installation



Pull repository
```bash
cd `echo $ROS_PACKAGE_PATH | awk -F: '{ print $1 }'`
git clone https://github.com/GSO-soslab/alpha_auv
cd alpha_auv
git submodule update --init --recursive
```
> Username password login via git command is not supported by github anymore. Use ssh keys or personal access tokens.

Install pip and setup python3 as default
```bash
sudo apt install python3-pip
```

Install cola2 requirements
```bash
sudo apt install ros-noetic-rosbridge-server ros-noetic-joy lm-sensors lcov
pip install ruamel.yaml
```

Install dependencies
```bash
cd `echo $ROS_PACKAGE_PATH | awk -F: '{ print $1 }'`
rosdep install --from-paths src --ignore-src --rosdistro ${ROS_DISTRO} -y
```

Install cola2_lib standalone library
```bash
mkdir ~/soslab_ws
git clone https://bitbucket.org/iquarobotics/cola2_lib.git
cd cola2_lib
mkdir build
cmake ..
make -j4
sudo make install
```
