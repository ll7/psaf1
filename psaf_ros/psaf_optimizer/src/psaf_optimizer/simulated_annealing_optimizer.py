#!/usr/bin/env python

import math
import numpy as np
import random
from psaf_scenario.scenario_runner_commonroad import ScenarioRunner
import rospy


class SimulatedAnnealingOptimizer:

    def __init__(self, step: float, alpha: float, parameter_range: list, it_count: int,
                 time_weight: float, quality_weight: float, file: str):
        """

        :param step: step size for exploring
        :param alpha: parameter for the cooling function
        :param parameter_range: list of tuples for each parameter, which defines its range:
                                e.g. parameter_range = [[1, 2], [0, 100]] means that there are two parameters
                                to be searched up on and their range is between 1 and 2, and 0 and 100
        :param it_count: number of iterations to search the optimum
        :param time_weight: weight of the duration time
        :param quality_weight: weight of the quality value
        :param file: file path and name to which the results shall be written
        # TODO: Add default values for parameters
        """
        self.scenario_runner = ScenarioRunner(init_rospy=True)
        self.step = step
        self.alpha = alpha
        self.parameter_range = parameter_range
        self.parameter_count = len(self.parameter_range)

        # start
        best_result, best_parameter_set = self.find_optimum(it_count, time_weight=time_weight,
                                                            quality_weight=quality_weight)
        sample_count = self.scenario_runner.sample_cnt

        # write results to file
        self._write_results_to_file(file, best_result, best_parameter_set, sample_count)

    def _set_params(self, params: np.ndarray):
        """
        Implements the functionality to set run parameters of the given problem space
        :param params: list of parameters
        """
        # TODO: Implement :D
        pass

    def _write_results_to_file(self, file: str, best_result: float, best_params: list, sample_count: int):
        """
        Write results to file
        :param file: file path and name
        :param best_result: best received result (calculated by evaluation metric)
        :param best_params: best received set of parameters
        :param sample_count: used sample count of the scenario runner
        """
        f = open(file, "w")
        f.write("Sample count: " + str(sample_count))
        f.write("Best result: "+str(best_result))
        best_param_str = "["
        for param in best_params:
            best_param_str += str(param) + ","
        best_param_str += "]"
        f.write("Best parameters: " + best_param_str)

    def _run_scenario(self, params: np.ndarray, time_weight: float = 1.0, quality_weight: float = 1.5):
        """
        Runs the test scenario with params
        :param params: list of the parameter set to test
        :param time_weight: weight of the duration time
        :param quality_weight: weight of the quality value
        :return: weighted quality measure
        """
        # set run parameters
        self._set_params(params)

        self.scenario_runner.init_scenario()
        self.scenario_runner.execute_scenario()
        route_quality = self.scenario_runner.evaluate_route_quality_cos()
        time_duration = self.scenario_runner.finish_time_stamp - self.scenario_runner.start_time_stamp

        return (route_quality * quality_weight) + (time_duration * time_weight)

    def _generate_initial_parameter_set(self):
        """
        Generates the initial parameter set, based on parameter range and count
        :return:
        """
        params = []
        for i in range(0, self.parameter_count):
            params.append(random.uniform(self.parameter_range[i][0], self.parameter_range[i][1]))

        print("Hallo Tobi")
        return np.array(params)

    def find_optimum(self, it_count, time_weight: float, quality_weight: float):
        """
        o = current_index, o' = neighbour_index, o_best = best_index
        b(o) = current_value, b(o_best) = best_value, b(o') = neighbour_value
        :param it_count: number of iterations
        :param time_weight: weight of the duration time
        :param quality_weight: weight of the quality value
        :return: best result
        """
        current_index = []
        best_result = []
        current_index = self._generate_initial_parameter_set()
        best_index = current_index.copy()
        best_value = float(self._run_scenario(params=best_index, time_weight=time_weight, quality_weight=quality_weight))
        current_value = best_value  # init
        for i in range(0, it_count):
            T = self._cooling_function(it_count, i)  # a
            neighbour_index = self._get_random_neighbour(current_index)  # b
            neighbour_value = float(self._run_scenario(params=neighbour_index, time_weight=time_weight,
                                                       quality_weight=quality_weight))
            if neighbour_value <= current_value:
                current_index = neighbour_index.copy()  # c1
                current_value = neighbour_value
            else:
                tmp = random.uniform(0, 1)
                probability = self._acceptance_probability(neighbour_value - current_value, T)
                if tmp > probability:
                    current_index = neighbour_index.copy()  # c2
                    current_value = neighbour_value
            if current_value < best_value:
                best_index = current_index
                best_value = current_value
            best_result = best_value
        return best_result, best_index

    def _cooling_function(self, i_max, i):
        return i_max * (self.alpha ** i)

    def _acceptance_probability(self, delta, t):
        # TODO: to be looked at
        return math.exp(delta / t * (-1))

    def _get_random_neighbour(self, index):
        neighbour_index = np.zeros(len(index))
        for i in range(0, len(index)):
            # do steps with step_size * (1% of total value range)
            value_range = abs(self.parameter_range[i][1] - self.parameter_range[i][0]) * 0.01
            d = random.uniform(-value_range * self.step, value_range * self.step)
            if (index[i] + d > self.parameter_range[i][1]) or (index[i] + d < self.parameter_range[i][0]):
                i = i - 1
            else:
                neighbour_index[i] = index[i] + d
        return neighbour_index


def main():
    # TODO
    pass


if __name__ == "__main__":
    try:
        main()
    except rospy.ROSInterruptException:
        pass
