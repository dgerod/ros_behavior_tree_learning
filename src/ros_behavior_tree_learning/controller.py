from interface import implements
from rik_utilities.activities import Task
from behavior_tree_learning.core.sbt import behavior_tree
from behavior_tree_learning.core.gp import GeneticParameters, GeneticSelectionMethods
from behavior_tree_learning.core.gp_sbt import BehaviorTreeLearner
from ros_behavior_tree_learning_comms.msg import State
from ros_behavior_tree_learning.gp_steps import GeneticProgrammingSteps
from ros_behavior_tree_learning.request import InteractiveStep
from ros_behavior_tree_learning.port_helpers import wait_port_until


class ControllerTask(implements(Task)):

    def __init__(self, settings_file_path, outputs_directory_path,
                 ports, publisher):

        self._settings_file = settings_file_path
        self._outputs_directory = outputs_directory_path
        self._parameters_path = ""

        self._step_port = ports.step
        self._bt_output_port = ports.bt_output
        self._fitness_input_port = ports.fitness_input
        self._state_publisher = publisher

    def initialize(self):
        return True

    def finalize(self):
        return True

    def step(self):

        print("step")
        self._state_publisher.publish(State(State.IDLE))
        if wait_port_until(self._step_port.request, InteractiveStep.STEP_START_EXECUTION):
            self._step_port.reply.push(True)
            success = self._execute_learning()

        # SEND_WORK_COMPLETED

    @staticmethod
    def _load_parameters(file_path):

        parameters = GeneticParameters()

        parameters.n_population = 16
        parameters.n_generations = 200
        parameters.ind_start_length = 8
        parameters.f_crossover = 0.5
        parameters.n_offspring_crossover = 2
        parameters.replace_crossover = False
        parameters.f_mutation = 0.5
        parameters.n_offspring_mutation = 2
        parameters.parent_selection = GeneticSelectionMethods.RANK
        parameters.survivor_selection = GeneticSelectionMethods.RANK
        parameters.f_elites = 0.1
        parameters.f_parents = parameters.f_elites
        parameters.mutate_co_offspring = False
        parameters.mutate_co_parents = True
        parameters.mutation_p_add = 0.4
        parameters.mutation_p_delete = 0.3
        parameters.allow_identical = False

        # add specific class for plot_parameters
        parameters.plot_fitness = False
        parameters.plot_best_individual = False
        parameters.plot_last_generation = False

        verbose = False
        return parameters, verbose

    def _prepare_bt_settings(self):

        behavior_tree.load_settings_from_file(self._settings_file)

    def _create_gp_steps(self, verbose):

        return GeneticProgrammingSteps(self._step_port,
                                       self._bt_output_port,
                                       self._fitness_input_port,
                                       self._state_publisher)

    def _execute_learning(self):

        print("execute_learning")

        self._prepare_bt_settings()
        parameters, verbose = self._load_parameters(self._parameters_path)
        steps = self._create_gp_steps(verbose)

        parameters.log_name = "btl_gp"
        bt_learner = BehaviorTreeLearner.from_steps(steps)
        return bt_learner.run(parameters, outputs_dir_path=self._outputs_directory, verbose=verbose)
