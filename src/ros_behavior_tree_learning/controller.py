from interface import implements
import yaml
from tasks_toolkit.activities import Task
from behavior_tree_learning.core.sbt import behavior_tree
from behavior_tree_learning.core.gp import GeneticParameters, GeneticSelectionMethods
from behavior_tree_learning.core.gp_sbt import BehaviorTreeLearner
from ros_behavior_tree_learning.connector import StatePublisher
from ros_behavior_tree_learning.gp_steps import GeneticProgrammingSteps
from ros_behavior_tree_learning.request import InteractiveStep
from ros_behavior_tree_learning.port_helpers import wait_port_until


class ControllerTask(implements(Task)):

    def __init__(self, bt_settings_file_path, gp_parameters_file_path, outputs_directory_path,
                 ports, publisher):

        self._bt_settings_path = bt_settings_file_path
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
        parameters.log_name = "btl-gp"
        verbose = False

        with open(file_path) as f:

            data = yaml.load(f, Loader=yaml.FullLoader)

            if "parameters" in data:

                if "n_population" in data["parameters"]:
                    parameters.n_population = data["parameters"]["n_population"]
                if "n_generations" in data["parameters"]:
                    parameters.n_generations = data["parameters"]["n_generations"]
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
                    parameters.plot_fitness = data["plot"]["fitness"]
                if "best_individual" in data["plot"]:
                    parameters.plot_best_individual = data["plot"]["best_individual"]
                if "last_generation" in data["plot"]:
                    parameters.plot_last_generation = data["plot"]["last_generation"]

            if "verbose" in data:
                verbose = data["verbose"]

        return parameters, verbose

    def _prepare_bt_settings(self):

        behavior_tree.load_settings_from_file(self._bt_settings_path)

    def _create_gp_steps(self, verbose):

        return GeneticProgrammingSteps(self._step_port,
                                       self._bt_output_port,
                                       self._fitness_input_port,
                                       self._state_publisher)

    def _execute_learning(self):

        print("execute_learning")

        self._prepare_bt_settings()
        parameters, verbose = self._load_parameters(self._gp_parameters_path)
        steps = self._create_gp_steps(verbose)

        parameters.log_name = "btl_gp"
        bt_learner = BehaviorTreeLearner.from_steps(steps)
        return bt_learner.run(parameters, outputs_dir_path=self._outputs_directory, verbose=verbose)
