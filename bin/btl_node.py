#!/usr/bin/env python

import os
import sys
import rospy
from ros_behavior_tree_learning.ros_node import start_node


def usage(argv):

    print("\n%s name bt_settings gp_parameters outputs_dir\n" % os.path.basename(argv[0]))


def main():

    if len(sys.argv) == 4:
        name = str(sys.argv[1])
        bt_settings_file = str(sys.argv[2])
        gp_parameters_file = str(sys.argv[3])
        outputs_dir = str(sys.argv[4])
    else:
        usage(sys.argv)
    try:
        start_node(name, bt_settings_file, gp_parameters_file, outputs_dir)
    
    except rospy.ROSInterruptException:
        pass


if __name__ == '__main__':
    main()
