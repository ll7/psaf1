#!/usr/bin/env python

import math
import numpy as np
import random
import datetime
import pathlib
from psaf_scenario.scenario_runner_commonroad import ScenarioRunner
import rospy
import sys
from dynamic_reconfigure import client
import dynamic_reconfigure
from enum import Enum
import json


class ParameterType(Enum):
    speed = 0
    acceleration = 1
    steering = 2


class SimulatedAnnealingOptimizer:

    def __init__(self, step: float, alpha: float, parameter_range: list, it_count: int, height: float,
                 time_weight: float, quality_weight: float, param_enum: ParameterType, sample_count: int, file: str,
                 timeout: int):
        """

        :param step: step size in percent of total value range, given in parameter_range
        :param alpha: parameter for the cooling function
        :param parameter_range: list of tuples for each parameter, which defines its range:
                                e.g. parameter_range = [[1, 2], [0, 100]] means that there are two parameters
                                to be searched up on and their range is between 1 and 2, and 0 and 100
        :param it_count: number of iterations to search the optimum
        :param time_weight: weight of the duration time
        :param quality_weight: weight of the quality value
        :param height: spawn height of car in the test scenario
        :param sample_count: sample count of scenario runner
        :param file: file of scenario runner
        :param timeout: timeout of scenario runner
        """
        self.scenario_runner = ScenarioRunner(init_rospy=False, height=height, sample_cnt=sample_count, route_file=file,
                                              timeout=timeout)
        self.logfile = None
        self.init_logfile()
        self.step = step
        self.alpha = alpha
        self.parameter_range = parameter_range
        self.parameter_count = len(self.parameter_range)
        self.param_enum: ParameterType = param_enum
        if not self._check_parameter_validity():
            rospy.logerr("The given parameter set is not valid")
            sys.exit(1)

        # start
        best_result, best_parameter_set = self.find_optimum(it_count, time_weight=time_weight,
                                                            quality_weight=quality_weight)
        sample_count = self.scenario_runner.sample_cnt

        # write results to file
        self._write_results_to_file(best_result, best_parameter_set, sample_count)

    def _check_parameter_validity(self):
        """
        Check if the given parameters match certain conditions.
        Has to be adjusted for every new parameter set, which is set to be optimized!
        :return: bool
        """
        if self.param_enum is ParameterType.speed or self.param_enum is ParameterType.acceleration:
            if len(self.parameter_range) != 3:
                return False

            for par_range in self.parameter_range:
                if par_range[0] < 0 or par_range[1] > 1:
                    return False
            return True

        elif self.param_enum is ParameterType.steering:
            pass
        else:
            pass

        return False

    def _set_params(self, params: np.ndarray):
        """
        Implements the functionality to set run parameters of the given problem space
        :param params: list of parameters
        """
        if self.param_enum is ParameterType.speed or self.param_enum is ParameterType.acceleration:
            self._set_params_pid(params)
        elif self.param_enum is ParameterType.steering:
            pass
        else:
            sys.exit(1)

    def _set_params_pid(self, params: np.ndarray):
        try:
            rec_client = dynamic_reconfigure.client.Client('/carla/ego_vehicle/ackermann_control', timeout=10)
            if self.param_enum is ParameterType.speed:
                rec_client.update_configuration({'speed_Kp': params[0], 'speed_Ki': params[1], 'speed_Kd': params[2]})
            else:
                rec_client.update_configuration({'accel_Kp': params[0], 'accel_Ki': params[1], 'accel_Kd': params[2]})
        except rospy.ROSException as e:
            print("Service not available: %s" % str(e))

    def _write_results_to_file(self, best_result: float, best_params: list, sample_count: int):
        """
        Write results to file
        :param best_result: best received result (calculated by evaluation metric)
        :param best_params: best received set of parameters
        :param sample_count: used sample count of the scenario runner
        """
        f = open(self.logfile + "_results.txt", "w")
        f.write("Sample count: " + str(sample_count) + "\n")
        f.write("Best result: " + str(best_result) + "\n")
        best_param_str = "["
        for param in best_params:
            best_param_str += str(param) + ","
        best_param_str += "]"
        f.write("Best parameters: " + best_param_str + "\n")

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
        current_index = self._generate_initial_parameter_set()
        best_index = current_index.copy()
        best_value = float(self._run_scenario(params=best_index, time_weight=time_weight, quality_weight=quality_weight))
        current_value = best_value  # init
        # start iterating
        for i in range(0, it_count):
            T = self._cooling_function(it_count, i)  # calculate temperature of current iteration
            neighbour_index = self._get_random_neighbour(current_index)  # get a new index
            neighbour_value = float(self._run_scenario(params=neighbour_index, time_weight=time_weight,
                                                       quality_weight=quality_weight))  # evaluate that index
            if neighbour_value <= current_value:  # compare these indices and replace with optimum
                current_index = neighbour_index.copy()
                current_value = neighbour_value
            else:
                tmp = random.uniform(0, 1)  # replace index anyway with probability p
                probability = self._acceptance_probability(neighbour_value - current_value, T)
                if tmp < probability:
                    current_index = neighbour_index.copy()
                    current_value = neighbour_value

            if current_value < best_value:  # save best
                best_index = current_index
                best_value = current_value
            # log iteration results
            self.log_to_file(current_value=current_value, current_index=current_index, best_value=best_value,
                             best_index=best_index)

        return best_value, best_index

    def _cooling_function(self, i_max, i):
        return i_max * (self.alpha ** i)

    def _acceptance_probability(self, delta, t):
        return math.exp(delta / t * (-1))

    def _get_random_neighbour(self, index):
        neighbour_index = np.zeros(len(index))
        i = 0
        while i < len(index):
            # do steps with step [%] * total value range
            value_range = abs(self.parameter_range[i][1] - self.parameter_range[i][0])
            d = random.uniform(-value_range * self.step, value_range * self.step)
            if (index[i] + d > self.parameter_range[i][1]) or (index[i] + d < self.parameter_range[i][0]):
                i = i - 1
            else:
                neighbour_index[i] = index[i] + d
            i += 1
        return neighbour_index

    def init_logfile(self):
        """
        creates .txt for logging the values which are getting optimized
        """

        current_date_and_time = datetime.datetime.now().strftime("%Y-%m-%d %H:%M:%S")
        # get destination path for the log file
        logs_path = str(pathlib.Path(__file__).parent.absolute().parents[1]) + "/logs"
        # create /logs folder if not already existent
        pathlib.Path(logs_path).mkdir(parents=False, exist_ok=True)
        # create name and file
        self.logfile = logs_path + "/" + str(current_date_and_time)
        file = open(self.logfile + ".txt", 'w')
        # write header for data
        file.write("Timestamp, Current Value, Current Index, Best Value, Best Index \n")
        file.close()

    def log_to_file(self, current_value, current_index, best_value, best_index):
        """
        :param current_value: value of the current iteration
        :param current_index: parameters of the current iteration
        :param best_value: value of the best iteration
        :param best_index: parameters of the best iteration
        :return:
        """
        current_date_and_time = datetime.datetime.now().strftime("%Y-%m-%d %H:%M:%S")
        file = open(self.logfile + ".txt", 'a')
        file.write(current_date_and_time + "," + str(current_value) + "," + str(current_index) + "," +
                   str(best_value) + "," + str(best_index) + "\n")
        file.close()


def main():
    # pid setup
    param_range = [[0, 1], [0, 1], [0, 1]]
    rospy.init_node('SimulatedAnnealingOptimizer', anonymous=True)
    step = rospy.get_param('~step', 0.05)
    alpha = rospy.get_param('~alpha', 0.5)
    parameter_range = rospy.get_param('~parameter_range', param_range)
    parameter_range = json.loads(parameter_range) # convert string to list
    it_count = rospy.get_param('~it_count', 0.5)
    quality_weight = rospy.get_param('~quality_weight', 1)
    time_weight = rospy.get_param('~time_weight', 1)
    param_enum = rospy.get_param('~param_enum', 'speed')
    height = rospy.get_param('~height', 0)
    sample_count = rospy.get_param('~sample_cnt', 100)
    file = rospy.get_param('~file', "minimal_example.debugpath")
    timeout = rospy.get_param('~timeout', "-1")

    optimizer = SimulatedAnnealingOptimizer(step=step, alpha=alpha, parameter_range=parameter_range, it_count=it_count,
                                            quality_weight=quality_weight, time_weight=time_weight,
                                            param_enum=ParameterType[param_enum], height=height,
                                            sample_count=sample_count, file=file, timeout=timeout)


if __name__ == "__main__":
    try:
        main()
    except rospy.ROSInterruptException:
        pass
