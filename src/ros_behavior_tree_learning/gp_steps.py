from interface import implements
from behavior_tree_learning.core.gp import AlgorithmSteps
from ros_behavior_tree_learning_comms.msg import State
from ros_behavior_tree_learning.request import InteractiveStep
from ros_behavior_tree_learning.port_helpers import wait_port, wait_port_until


class GeneticProgrammingSteps(implements(AlgorithmSteps)):

    _WAIT_REFRESH_TIME = 1.0

    def __init__(self, step_ports, bt_port, fitness_port, state_publisher, verbose=False):

        self._step_ports = step_ports
        self._bt_output_port = bt_port
        self._fitness_input_port = fitness_port
        self._state_publisher = state_publisher

    def execution_started(self):
        print("notify_start")
        self._state_publisher.publish(State(State.RUNNING))

    def execute_generation(self, generation):

        print("executing_generation: %d" % generation)
        self._state_publisher.publish(State(State.RUNNING))

        wait_port_until(self._step_ports.request, InteractiveStep.STEP_EXECUTE_GENERATION, self._WAIT_REFRESH_TIME)
        self._state_publisher.publish(State(State.RUNNING))
        self._step_ports.reply.push(True)

    def current_population(self, population):
        print("current_population")

    def calculate_fitness(self, individual, verbose):

        print("calculate_fitness")
        self._state_publisher.publish(State(State.CALCULATING_FITNESS))

        self._send_individual(individual)
        fitness = self._wait_fitness_result()

        self._state_publisher.publish(State(State.RUNNING))
        return fitness

    def plot_individual(self, path, plot_name, individual):
        print("plot_individual")

    def more_generations(self, generation, last_generation, error):

        print("more_generations: %d/%d" % (generation, last_generation))
        self._state_publisher.publish(State(State.CHECKING_NEXT_GENERATION))

        wait_port_until(self._step_ports.request, InteractiveStep.STEP_NEXT_GENERATION, self._WAIT_REFRESH_TIME)
        another_generation = True if generation < last_generation else False
        self._step_ports.reply.push(another_generation)

        self._state_publisher.publish(State(State.RUNNING))

    def execution_completed(self):
        print("execution_completed")
        self._state_publisher.publish(State(State.COMPLETED))

    def _send_individual(self, individual):

        wait_port_until(self._step_ports.request, InteractiveStep.STEP_POP_BEHAVIOR_TREE, self._WAIT_REFRESH_TIME)
        text = ", ".join(individual)
        self._bt_output_port.push(text)
        self._step_ports.reply.push(True)

    def _wait_fitness_result(self):

        wait_port_until(self._step_ports.request, InteractiveStep.STEP_PUSH_FITNESS, self._WAIT_REFRESH_TIME)
        fitness = wait_port(self._fitness_input_port, self._WAIT_REFRESH_TIME)
        self._step_ports.reply.push(True)
        return fitness
