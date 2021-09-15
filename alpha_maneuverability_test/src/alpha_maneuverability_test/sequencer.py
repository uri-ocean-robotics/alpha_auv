import os
import yaml
import rospy
import geometry_msgs.msg
import datetime
import copy
import time
from .recorder import Recorder


class Sequencer:
    def __init__(self):
        self._check_ros()

        self._config_file = rospy.get_param("~config_file")

        self._thrust_pub = rospy.Publisher("thrust_cmd", geometry_msgs.msg.Vector3, queue_size=10)

        self.config = None

        self._read_config()

        self._check_config()

        self.recorder = Recorder(
            bag_name=os.path.basename(self._config_file) + "."
        )

    @staticmethod
    def _check_ros():
        try:
            rospy.get_time()
        except rospy.ROSInitException as e:
            raise RuntimeError(e)

    def _read_config(self):
        with open(self._config_file, 'r') as stream:
            try:
                self.config = yaml.safe_load(stream)
            except yaml.YAMLError as e:
                raise e

    def _check_config(self):
        if 'sequence' not in self.config:
            raise Exception("sequence has to be present")

        if not len(self.config['sequence']) > 0:
            raise Exception("sequence has to be a non-empty list")

        for act in self.config['sequence']:
            if "duration" not in act:
                raise Exception("duration has to be present")

    def _act(self):
        default_act = {
            "main": 1500,
            "vertical": 1500,
            "horizontal": 1500
        }

        current_act = copy.deepcopy(default_act)

        ts = geometry_msgs.msg.Vector3()
        for act in self.config['sequence']:
            if "main" in act:
                current_act["main"] = act["main"]

            if "vertical" in act:
                current_act["vertical"] = act["vertical"]

            if "horizontal" in act:
                current_act["horizontal"] = act["horizontal"]

            ts.x = current_act["main"]
            ts.y = current_act["horizontal"]
            ts.z = current_act["vertical"]

            end_act = datetime.datetime.now() + datetime.timedelta(0, act["duration"])
            while datetime.datetime.now() < end_act:
                self._thrust_pub.publish(ts)
                rospy.sleep(rospy.Duration(0, 1000000))

        ts.x = default_act["main"]
        ts.y = default_act["horizontal"]
        ts.z = default_act["vertical"]
        end_act = datetime.datetime.now() + datetime.timedelta(0, 2)
        while datetime.datetime.now() < end_act:
            self._thrust_pub.publish(ts)
            rospy.sleep(rospy.Duration(0, 1000000))

    def start(self):
        self.recorder.start_recording()
        self._act()
        time.sleep(10)
        self.recorder.stop_recording()

