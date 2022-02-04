#!/usr/bin/env python

import sys
import rospy
from ros_behavior_tree_learning.ros_node import start_node


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
        start_node(name, settings_file, outputs_dir)
    
    except rospy.ROSInterruptException:
        pass


if __name__ == '__main__':
    main()
