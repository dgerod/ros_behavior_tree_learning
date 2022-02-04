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
        return response.success

    except rospy.ServiceException as e:
        print("Service call failed: %s" % e)


def _do_execute_generation():

    rospy.wait_for_service(SERVICE_NAME)

    try:
        request = GpInteractiveCtrlRequest()
        request.step = GpInteractiveCtrlRequest.STEP_EXECUTE_GENERATION
        service = rospy.ServiceProxy(SERVICE_NAME, GpInteractiveCtrl)
        response = service(request)
        return response.success, response.next_generation.continue_

    except rospy.ServiceException as e:
        print("Service call failed: %s" % e)


def _do_pop_behavior_tree():

    rospy.wait_for_service(SERVICE_NAME)

    try:
        request = GpInteractiveCtrlRequest()
        request.step = GpInteractiveCtrlRequest.STEP_POP_BEHAVIOR_TREE
        service = rospy.ServiceProxy(SERVICE_NAME, GpInteractiveCtrl)
        response = service(request)

        print("bt: %s", response.pop_bt.bt)
        return response.success, response.pop_bt

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
        return response.success

    except rospy.ServiceException as e:
        print("Service call failed: %s" % e)


def _do_next_generation():

    rospy.wait_for_service(SERVICE_NAME)

    try:
        request = GpInteractiveCtrlRequest()
        request.step = GpInteractiveCtrlRequest.STEP_NEXT_GENERATION
        service = rospy.ServiceProxy(SERVICE_NAME, GpInteractiveCtrl)
        response = service(request)

        print("another: %b", response.next_generation.continue_)
        return response.success, response.next_generation.continue_

    except rospy.ServiceException as e:
        print("Service call failed: %s" % e)


def usage(argv):

    print("\n%s STEP [bt fitness]\n" % os.path.basename(argv[0]))
    print("STEP: start (start execution of the GP algorithm)")
    print("      execute (initialize the GP algorithm)")
    print("      pop (retrieve BT)")
    print("      push bt fitness (provide calculated fitness)")
    print("      next (retrieve if there is another generation)")


if __name__ == "__main__":

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
        success = _do_start_execution()
    elif step == 'execute':
        success, continue_ = _do_execute_generation()
    elif step == "pop":
        success, bt = _do_pop_behavior_tree()
    elif step == "push":
        success = _do_push_fitness(bt, fitness)
    elif step == 'next':
        success, continue_ = _do_next_generation()
    else:
        raise ValueError("Unknown step")
