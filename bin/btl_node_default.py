#!/usr/bin/env python

import os
import rospy

from ros_behavior_tree_learning.ros_node import start_node


_this_file_path = os.path.abspath(__file__)
_EXAMPLES_DIRECTORY = os.path.dirname(os.path.dirname(_this_file_path))


def main():

    try:
        start_node("btl_gp",
                   os.path.join(_EXAMPLES_DIRECTORY, "duplo_settings.yaml"),
                   os.path.join(_EXAMPLES_DIRECTORY, "results"))
    
    except rospy.ROSInterruptException:
        pass


if __name__ == '__main__':
    main()
