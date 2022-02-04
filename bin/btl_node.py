#!/usr/bin/env python

import os
import sys
import rospy

from ros_behavior_tree_learning.ros_node import start_node


_this_file_path = os.path.abspath(__file__)
_CURRENT__DIRECTORY = os.path.dirname(_this_file_path)


def usage(argv):

    print("\n%s name settings_file outputs_dir\n" % os.path.basename(argv[0]))


def main():

    if len(sys.argv) == 3:
        name = str(sys.argv[1])
        settings_file = str(sys.argv[2])
        outputs_dir = str(sys.argv[3])
    else:
        usage(sys.argv)
    try:
        start_node("btl_gp",
                   os.path.join(_CURRENT__DIRECTORY, "bt_settings.yaml"),
                   _CURRENT__DIRECTORY)
    
    except rospy.ROSInterruptException:
        pass


if __name__ == '__main__':
    main()
