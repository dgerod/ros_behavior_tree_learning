import os
import logging
import yaml
from interface import implements

from tasks_toolkit.activities import Task
from behavior_tree_learning.gp import GeneticParameters, GeneticSelectionMethods, TraceConfiguration

from behavior_tree_learning.cbt_learning import configure_cbt_checker
from behavior_tree_learning.cbt_learning import BehaviorTreeLearner as CbtTreeLearner
from behavior_tree_learning.cbt_learning import BehaviorNodeFactory as CbtNodeFactory

from behavior_tree_learning.sbt_learning import BehaviorTreeLearner as SbtTreeLearner
from behavior_tree_learning.sbt_learning import BehaviorNodeFactory as SbtNodeFactory

from ros_behavior_tree_learning.connector import StatePublisher
from ros_behavior_tree_learning.gp_steps import GeneticProgrammingSteps
from ros_behavior_tree_learning.request import InteractiveStep
from ros_behavior_tree_learning.port_helpers import wait_port_until


def _configure_logger(level, directory_path, name):

    for handler in logging.root.handlers[:]:
        logging.root.removeHandler(handler)

    file_path = os.path.join(directory_path, name + '.log')

    try:
        os.mkdir(directory_path)
    except:
        pass

    logging.basicConfig(filename=file_path,
                        format='%(filename)s: %(message)s')
    logging.getLogger("gp").setLevel(level)


class ControllerTask(implements(Task)):

    def __init__(self, bt_type, bt_settings_file_path, gp_parameters_file_path, outputs_directory_path,
                 ports, publisher):

        if bt_type == "sbt":
            self._bt_settings_path = bt_settings_file_path
            self._bt_initial_state_path = ""

        else:
            self._bt_settings_path = bt_settings_file_path + "_settings.yaml"
            self._bt_initial_state_path = bt_settings_file_path + "_initial_state.yaml"

            configure_cbt_checker({"app_path": "/home/ubuntu/ros_ws/src/cbt_checker/bin",
                                   "dependencies_path": "/home/ubuntu/ros_ws/src/cbt_checker/dependencies/limboole/limboole",
                                   "temporary_directory": "/home/ubuntu/ros_ws/src/behavior_tree_learning/tmp",
                                   "logger": "NONE"})

        self._bt_type = bt_type
        self._gp_parameters_path = gp_parameters_file_path
        self._outputs_directory = outputs_directory_path

        self._step_port = ports.step
        self._bt_output_port = ports.bt_output
        self._fitness_input_port = ports.fitness_input
        self._state_publisher = publisher

    def initialize(self):
        return True

    def finalize(self):
        return True

    def step(self):

        self._state_publisher.send(StatePublisher.States.NOT_STARTED)

        if wait_port_until(self._step_port.request, InteractiveStep.STEP_START_EXECUTION):
            self._state_publisher.send(StatePublisher.States.RUNNING)
            self._step_port.reply.push(True)
            self._execute_learning()

        # SEND_WORK_COMPLETED

    @staticmethod
    def _map_selection_method(method, parameter_name):
        if method == "ELITISM":
            return GeneticSelectionMethods.ELITISM
        elif method == "TOURNAMENT":
            return GeneticSelectionMethods.TOURNAMENT
        elif method == "RANK":
            return GeneticSelectionMethods.RANK
        elif method == "RANDOM":
            return GeneticSelectionMethods.RANDOM
        elif method == "RANDOM":
            return GeneticSelectionMethods.RANDOM
        else:
            raise ValueError("Selection method in [%s] does not exist" % parameter_name)

    @staticmethod
    def _load_parameters(file_path):

        parameters = GeneticParameters()
        parameters.log_name = "btl_gp"
        verbose = False

        trace_configuration = TraceConfiguration()

        with open(file_path) as f:

            data = yaml.load(f, Loader=yaml.FullLoader)

            if "parameters" in data:

                if "n_population" in data["parameters"]:
                    parameters.n_population = data["parameters"]["n_population"]
                if "n_generations" in data["parameters"]:
                    parameters.n_generations = data["parameters"]["n_generations"]
                if "fitness_threshold" in data["parameters"]:
                    parameters.fitness_threshold = data["parameters"]["fitness_threshold"]

                if "ind_start_length" in data["parameters"]:
                    parameters.ind_start_length = data["parameters"]["ind_start_length"]
                if "f_crossover" in data["parameters"]:
                    parameters.f_crossover = data["parameters"]["f_crossover"]
                if "n_offspring_crossover" in data["parameters"]:
                    parameters.n_offspring_crossover = data["parameters"]["n_offspring_crossover"]
                if "replace_crossover" in data["parameters"]:
                    parameters.replace_crossover = data["parameters"]["replace_crossover"]
                if "f_mutation" in data["parameters"]:
                    parameters.f_mutation = data["parameters"]["f_mutation"]
                if "n_offspring_mutation" in data["parameters"]:
                    parameters.n_offspring_mutation = data["parameters"]["n_offspring_mutation"]

                if "parent_selection" in data["parameters"]:
                    method = data["parameters"]["parent_selection"]
                    parameters.parent_selection = ControllerTask._map_selection_method(method, "parent_selection")

                if "survivor_selection" in data["parameters"]:
                    method = data["parameters"]["survivor_selection"]
                    parameters.survivor_selection = ControllerTask._map_selection_method(method, "survivor_selection")

                if "f_elites" in data["parameters"]:
                    parameters.f_elites = data["parameters"]["f_elites"]
                if "f_parents" in data["parameters"]:
                    parameters.f_parents = data["parameters"]["f_parents"]
                if "mutate_co_offspring" in data["parameters"]:
                    parameters.mutate_co_offspring = data["parameters"]["mutate_co_offspring"]
                if "mutate_co_parents" in data["parameters"]:
                    parameters.mutate_co_parents = data["parameters"]["mutate_co_parents"]
                if "mutation_p_add" in data["parameters"]:
                    parameters.mutation_p_add = data["parameters"]["mutation_p_add"]
                if "mutation_p_delete" in data["parameters"]:
                    parameters.mutation_p_delete = data["parameters"]["mutation_p_delete"]
                if "allow_identical" in data["parameters"]:
                    parameters.allow_identical = data["parameters"]["allow_identical"]

            if "plot" in data:

                if "fitness" in data["plot"]:
                    trace_configuration.plot_fitness = data["plot"]["fitness"]
                if "best_individual" in data["plot"]:
                    trace_configuration.plot_best_individual = data["plot"]["best_individual"]
                if "last_generation" in data["plot"]:
                    trace_configuration.plot_last_generation = data["plot"]["last_generation"]

            if "verbose" in data:
                verbose = data["verbose"]

        return parameters, trace_configuration, verbose

    def _prepare_bt_settings(self):

        def make_nodes(text, world, verbose=False):
            print("Node: %s" % text)
            return None

        conditions = []
        initial_state = ""

        if self._bt_type == "sbt":
            SbtNodeFactory().from_bt_settings(self._bt_settings_path, make_nodes)
        else:
            _, conditions = CbtNodeFactory.from_settings(self._bt_settings_path, make_nodes)

            with open(self._bt_initial_state_path) as f:
                bt_settings = yaml.load(f, Loader=yaml.FullLoader)
                initial_state = bt_settings['initial_state']

        return conditions, initial_state

    def _create_gp_steps(self, verbose):

        return GeneticProgrammingSteps(self._step_port,
                                       self._bt_output_port,
                                       self._fitness_input_port,
                                       self._state_publisher,
                                       verbose=verbose)

    def _create_learner(self, conditions, initial_state, steps):

        if self._bt_type == "sbt":
            bt_learner = SbtTreeLearner.from_steps(steps)
        else:
            bt_learner = CbtTreeLearner.from_steps(conditions, initial_state, steps)

        return bt_learner

    def _execute_learning(self):

        print("execute_learning")

        conditions, initial_state = self._prepare_bt_settings()
        parameters, trace_configuration, verbose = self._load_parameters(self._gp_parameters_path)
        steps = self._create_gp_steps(verbose)

        if verbose:
            _configure_logger(logging.DEBUG, self._outputs_directory, "btl")

        print("BT Conditions")
        print(conditions)
        print("BT Initial State")
        print(initial_state)
        print("GP Parameters")
        print(parameters)
        print("GP Trace")
        print(trace_configuration)

        bt_learner = self._create_learner(conditions, initial_state, steps)
        return bt_learner.run(parameters, outputs_dir_path=self._outputs_directory,
                              trace_conf=trace_configuration, verbose=verbose)
