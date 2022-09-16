#   This file is part of ALPHA AUV project.
#
#   This project is free software: you can redistribute it and/or modify
#   it under the terms of the GNU General Public License as published by
#   the Free Software Foundation, either version 3 of the License, or
#   (at your option) any later version.
#
#   This project is distributed in the hope that it will be useful,
#   but WITHOUT ANY WARRANTY; without even the implied warranty of
#   MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
#   GNU General Public License for more details.
#
#   You should have received a copy of the GNU General Public License
#   along with the project.  If not, see <https://www.gnu.org/licenses/>.
#
#   Author: Emir Cem Gezer
#   Email: emircem@uri.edu;emircem.gezer@gmail.com
#   Year: 2022
#
#   Copyright (C) 2022 Smart Ocean Systems Laboratory

import roslaunch
from time import strftime
from os.path import expanduser


class Recorder:
    def __init__(self, bag_name=""):
        self.node = None
        self.launch = None
        self.process = None

        self.target_path = expanduser("~") + "/bags"
        self.bag_name = bag_name
        self._setup_recording()

    def _setup_recording(self):
        package = 'rosbag'
        executable = 'record'

        self.bag_name += strftime("%Y-%m-%dT%H-%M-%S") + ".bag"

        target = self.target_path + "/" + self.bag_name

        self.node = roslaunch.core.Node(
            package,
            executable,
            args='-a -O {0}'.format(target)
        )

        self.launch = roslaunch.scriptapi.ROSLaunch()

    def start_recording(self):
        self.launch.start()
        self.process = self.launch.launch(self.node)

    def stop_recording(self):
        self.process.stop()
