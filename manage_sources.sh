#!/bin/bash

ALL_SOURCE_DIRS=`find . -name "package.xml" -exec dirname {} \;`

SIMULATION_DIRS="
alpha_controller
alpha_localization
alpha_description
alpha_teleop
alpha_simulator
alpha_msgs
"

function setup_simulation() {
  for src in ${ALL_SOURCE_DIRS[@]} ; do
    for d in ${SIMULATION_DIRS[@]} ; do
      echo $src | grep $d -q
      if [ $? -ne 0 ] ; then
        touch $src/CATKIN_IGNORE
      fi
    done
  done
}

function setup_vehicle() {
  find . -name "CATKIN_IGNORE" -exec rm {} \;
}

$1