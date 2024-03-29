import rospy
from tasks_toolkit.activities import ActivityFactory
from ros_behavior_tree_learning.connector import Connector
from ros_behavior_tree_learning.controller import ControllerTask


class GpNode:

    def __init__(self, name, bt_type, settings_file_path, parameters_file_path, outputs_directory_path):

        self._connector = Connector(name)
        self._controller = ControllerTask(bt_type, settings_file_path, parameters_file_path, outputs_directory_path,
                                          self._connector.ports(), self._connector.publisher())
        self._activity = ActivityFactory().make_activity(ActivityFactory.ActivityType.NORMAL, self._controller)
        self._activity.start()

    def stop(self):
        self._activity.stop()


def start_node(name, bt_type, bt_settings_path, gp_parameters_path, outputs_directory_path=""):

    rospy.init_node(name)

    try:
        node = GpNode(name, bt_type, bt_settings_path, gp_parameters_path, outputs_directory_path)
        rospy.spin()
    except rospy.ROSInterruptException:
        node.stop()

