import rospy
from tasks_toolkit.activities import ActivityFactory
from ros_behavior_tree_learning.connector import Connector
from ros_behavior_tree_learning.controller import ControllerTask


class GpNode:

    def __init__(self, name, settings_file_path, outputs_directory_path):

        self._connector = Connector(name)
        self._controller = ControllerTask(settings_file_path, outputs_directory_path,
                                          self._connector.ports(), self._connector.publisher())
        self._activity = ActivityFactory().make_activity(ActivityFactory.ActivityType.NORMAL, self._controller)
        self._activity.start()

    def stop(self):
        self._activity.stop()


def start_node(name, settings_path, outputs_directory_path=""):

    rospy.init_node(name)

    try:
        node = GpNode(name, settings_path, outputs_directory_path)
        rospy.spin()
    except rospy.ROSInterruptException:
        node.stop()

