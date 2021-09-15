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
