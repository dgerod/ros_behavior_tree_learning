#!/usr/bin/env python

import os
import sys
import rospy
from ros_behavior_tree_learning_comms.srv import GpInteractiveCtrl, GpInteractiveCtrlRequest, GpInteractiveCtrlResponse
from ros_behavior_tree_learning_comms.msg import NextGeneration, PopBehaviorTree, PushFitness


SERVICE_NAME = '/btl_gp/do_step'


def _do_start_execution():

    rospy.wait_for_service(SERVICE_NAME)

    try:
        request = GpInteractiveCtrlRequest()
        request.step = GpInteractiveCtrlRequest.STEP_START_EXECUTION

        service = rospy.ServiceProxy(SERVICE_NAME, GpInteractiveCtrl)
        response = service(request)

        print("Succeed: %s" % response.success)

    except rospy.ServiceException as e:
        print("Service call failed: %s" % e)


def _do_execute_generation():

    rospy.wait_for_service(SERVICE_NAME)

    try:
        request = GpInteractiveCtrlRequest()
        request.step = GpInteractiveCtrlRequest.STEP_EXECUTE_GENERATION

        service = rospy.ServiceProxy(SERVICE_NAME, GpInteractiveCtrl)
        response = service(request)

        print("Succeed: %s" % response.success)

    except rospy.ServiceException as e:
        print("Service call failed: %s" % e)


def _do_pop_behavior_tree():

    rospy.wait_for_service(SERVICE_NAME)

    try:
        request = GpInteractiveCtrlRequest()
        request.step = GpInteractiveCtrlRequest.STEP_POP_BEHAVIOR_TREE

        service = rospy.ServiceProxy(SERVICE_NAME, GpInteractiveCtrl)
        response = service(request)

        print("Success: %s, BT: %s" % (response.success, response.pop_bt.bt))

    except rospy.ServiceException as e:
        print("Service call failed: %s" % e)


def _do_push_fitness(bt, fitness):

    rospy.wait_for_service(SERVICE_NAME)

    try:
        request = GpInteractiveCtrlRequest()
        request.step = GpInteractiveCtrlRequest.STEP_PUSH_FITNESS
        request.push_fitness.bt = bt
        request.push_fitness.fitness = fitness

        service = rospy.ServiceProxy(SERVICE_NAME, GpInteractiveCtrl)
        response = service(request)

        print("Succeed: %s" % response.success)

    except rospy.ServiceException as e:
        print("Service call failed: %s" % e)


def _do_next_generation():

    rospy.wait_for_service(SERVICE_NAME)

    try:
        request = GpInteractiveCtrlRequest()
        request.step = GpInteractiveCtrlRequest.STEP_NEXT_GENERATION

        service = rospy.ServiceProxy(SERVICE_NAME, GpInteractiveCtrl)
        response = service(request)

        print("Succeed: %s, Another generation: %s"
              % (response.success, response.next_generation.continue_))

    except rospy.ServiceException as e:
        print("Service call failed: %s" % e)


def usage(argv):

    print("\n%s step [bt fitness]\n" % os.path.basename(argv[0]))
    print("step: start (start execution of the GP algorithm)")
    print("      next (retrieve if there is another generation)")
    print("      execute (start a new generation)")
    print("      pop (retrieve BT)")
    print("      push bt fitness (provide calculated fitness)")


def main():

    if len(sys.argv) == 2:
        step = str(sys.argv[1])
    elif len(sys.argv) == 4:
        step = str(sys.argv[1])
        bt = str(sys.argv[2])
        fitness = float(sys.argv[3])
    else:
        usage(sys.argv)
        sys.exit(1)

    if step == 'start':
        _do_start_execution()
    elif step == 'next':
        _do_next_generation()
    elif step == 'execute':
        _do_execute_generation()
    elif step == "pop":
        _do_pop_behavior_tree()
    elif step == "push":
        _do_push_fitness(bt, fitness)
    else:
        raise ValueError("Unknown step")


if __name__ == "__main__":
    main()
