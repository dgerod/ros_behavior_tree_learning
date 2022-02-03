#!/usr/bin/env python

import sys
import rospy
from behavior_tree_learning.srv import GpInteractiveCtrl, GpInteractiveCtrlRequest, GpInteractiveCtrlResponse
from behavior_tree_learning.msg import NextGeneration, PopBehaviorTree, PushFitness
from behavior_tree_learning.ros.request import InteractiveStep


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
        return response.success, response.next_generation.continue_

    except rospy.ServiceException as e:
        print("Service call failed: %s" % e)


def usage(argv):

    print("%s [step] {bt fitness}" % argv[0])
    print("step: start generation pop push next")


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
