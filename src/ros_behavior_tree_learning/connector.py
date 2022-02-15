from collections import namedtuple
import rospy
from tasks_toolkit.connections.ports import DataStorageLock
from tasks_toolkit.connections.ports import BufferStorageLock, InputBufferPort, OutputBufferPort
from ros_behavior_tree_learning_comms.srv import GpInteractiveCtrl, GpInteractiveCtrlRequest, GpInteractiveCtrlResponse
from ros_behavior_tree_learning_comms.msg import NextGeneration, PopBehaviorTree, PushFitness, State
from ros_behavior_tree_learning.request import InteractiveStep
from ros_behavior_tree_learning.port_helpers import wait_port


BidirectionalPort = namedtuple('BidirectionalPort', 'request reply')


class StatePublisher:

    class States:

        NOT_STARTED = 1
        RUNNING = 2
        WAIT_EXECUTE_GENERATION = 3
        WAIT_POP_BT = 4
        WAIT_PUSH_FITNESS = 5
        WAIT_ANOTHER_GENERATION = 6

    def __init__(self, publisher, converter):
        self._converter = converter
        self._publisher = publisher

    def send(self, state):
        new_state = self._converter(state)
        self._publisher.publish(new_state)


class Connector:

    PortConnection = namedtuple('PortConnection', 'input output')
    ExportedPorts = namedtuple('Ports', 'step bt_output fitness_input')

    def __init__(self, name: str):

        self._ports = self._create_ports()

        service_name = name + "/do_step"
        self._interactive_service = rospy.Service(service_name, GpInteractiveCtrl, self._interactive_service)
        publisher_name = name + "/state"

        self._state_storage = DataStorageLock()
        self._state_publisher = StatePublisher(rospy.Publisher(publisher_name, State, queue_size=10),
                                               self._update_state)

    def ports(self):
        return Connector.ExportedPorts(BidirectionalPort(self._ports["step_request"].input,
                                                         self._ports["step_reply"].output),
                                       self._ports["bt"].output,
                                       self._ports["fitness"].input)

    def publisher(self):
        return self._state_publisher

    @staticmethod
    def _create_ports():

        storage = BufferStorageLock()
        step_request_connection = Connector.PortConnection(InputBufferPort(storage), OutputBufferPort(storage))
        storage = BufferStorageLock()
        step_reply_connection = Connector.PortConnection(InputBufferPort(storage), OutputBufferPort(storage))

        storage = BufferStorageLock()
        bt_connection = Connector.PortConnection(InputBufferPort(storage), OutputBufferPort(storage))
        storage = BufferStorageLock()
        fitness_connection = Connector.PortConnection(InputBufferPort(storage), OutputBufferPort(storage))

        return {'step_request': step_request_connection, 'step_reply': step_reply_connection,
                'bt': bt_connection, 'fitness': fitness_connection}

    def _update_state(self, state):

        if state == StatePublisher.States.NOT_STARTED:
            new_state = State.IDLE
        elif state == StatePublisher.States.RUNNING:
            new_state = State.RUNNING
        elif state == StatePublisher.States.WAIT_EXECUTE_GENERATION:
            new_state = State.CHECKING_NEXT_GENERATION
        elif state == StatePublisher.States.WAIT_POP_BT:
            new_state = State.CALCULATING_FITNESS
        elif state == StatePublisher.States.WAIT_PUSH_FITNESS:
            new_state = State.CALCULATING_FITNESS
        elif state == StatePublisher.States.WAIT_ANOTHER_GENERATION:
            new_state = State.CHECKING_NEXT_GENERATION
        else:
            raise ValueError("Unknown state")

        self._state_storage.write(state)
        return new_state

    def _interactive_service(self, request):

        print("Connector::_interactive_service: %s" % request.step)

        if request.step == GpInteractiveCtrlRequest.STEP_START_EXECUTION:
            succeed = self._do_start_execution()
            return GpInteractiveCtrlResponse(succeed, NextGeneration(), PopBehaviorTree())
        elif request.step == GpInteractiveCtrlRequest.STEP_EXECUTE_GENERATION:
            succeed = self._do_execute_generation()
            return GpInteractiveCtrlResponse(succeed, NextGeneration(), PopBehaviorTree())
        elif request.step == GpInteractiveCtrlRequest.STEP_POP_BEHAVIOR_TREE:
            succeed, bt = self._do_pop_bt()
            return GpInteractiveCtrlResponse(succeed, NextGeneration(), PopBehaviorTree(bt))
        elif request.step == GpInteractiveCtrlRequest.STEP_PUSH_FITNESS:
            succeed = self._do_push_fitness(request.push_fitness.bt, request.push_fitness.fitness)
            return GpInteractiveCtrlResponse(succeed, NextGeneration(), PopBehaviorTree())
        elif request.step == GpInteractiveCtrlRequest.STEP_NEXT_GENERATION:
            succeed, another_generation = self._do_next_generation()
            return GpInteractiveCtrlResponse(succeed, NextGeneration(another_generation), PopBehaviorTree())
        else:
            return GpInteractiveCtrlResponse(False, NextGeneration(), PopBehaviorTree())

    def _do_start_execution(self):

        print("WaitConnector::_do_start_execution")

        if self._state_storage.read() != StatePublisher.States.NOT_STARTED:
            print("ERROR: Not in expected state")
            return False

        self._ports["step_request"].output.push(InteractiveStep.STEP_START_EXECUTION)
        _ = wait_port(self._ports["step_reply"].input)

        print("done")
        return True

    def _do_execute_generation(self):

        print("Connector::_do_execute_generation")

        if self._state_storage.read() != StatePublisher.States.WAIT_EXECUTE_GENERATION:
            print("ERROR: Not in expected state")
            return False

        self._ports["step_request"].output.push(InteractiveStep.STEP_EXECUTE_GENERATION)
        _ = wait_port(self._ports["step_reply"].input)

        print("done")
        return True

    def _do_pop_bt(self):

        print("Connector::_do_pop_bt")

        if self._state_storage.read() != StatePublisher.States.WAIT_POP_BT:
            print("ERROR: Not in expected state")
            return False, ""

        self._ports["step_request"].output.push(InteractiveStep.STEP_POP_BEHAVIOR_TREE)
        bt = wait_port(self._ports["bt"].input)
        _ = wait_port(self._ports["step_reply"].input)

        print("bt: %s" % bt)
        return True, bt

    def _do_push_fitness(self, bt, fitness):

        print("Connector::_do_push_fitness")

        if self._state_storage.read() != StatePublisher.States.WAIT_PUSH_FITNESS:
            print("ERROR: Not in expected state")
            return False

        print("bt: %s" % bt)
        print("fitness: %s" % fitness)

        self._ports["step_request"].output.push(InteractiveStep.STEP_PUSH_FITNESS)
        self._ports["fitness"].output.push(fitness)
        _ = wait_port(self._ports["step_reply"].input)

        print("done")
        return True

    def _do_next_generation(self):

        print("Connector::_do_next_generation")

        if self._state_storage.read() != StatePublisher.States.WAIT_ANOTHER_GENERATION:
            print("ERROR: Not in expected state")
            return False, False

        self._ports["step_request"].output.push(InteractiveStep.STEP_NEXT_GENERATION)
        another_generation = wait_port(self._ports["step_reply"].input)

        print("more generations: %s" % another_generation)
        return True, another_generation
