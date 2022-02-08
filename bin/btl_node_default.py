#!/usr/bin/env python

import os
import rospy

from ros_behavior_tree_learning.ros_node import start_node


_CURRENT_DIRECTORY = os.path.dirname(os.path.abspath(__file__))
_EXAMPLES_DIRECTORY = os.path.join(os.path.dirname(_CURRENT_DIRECTORY), "examples")


def main():

    try:
        start_node("btl_gp",
                   os.path.join(_EXAMPLES_DIRECTORY, "duplo", "bt_settings.yaml"),
                   os.path.join(_EXAMPLES_DIRECTORY, "duplo", "gp_parameters.yaml"),
                   os.path.join(_EXAMPLES_DIRECTORY, "results"))
    
    except rospy.ROSInterruptException:
        pass


if __name__ == '__main__':
    main()
