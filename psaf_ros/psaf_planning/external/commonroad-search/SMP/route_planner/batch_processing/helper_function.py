import fnmatch
import logging
import os
import pickle
import time
from datetime import datetime
from typing import Tuple

import matplotlib as mpl

try:
    mpl.use('Qt5Agg')
    import matplotlib.pyplot as plt
except ImportError:
    mpl.use('TkAgg')
    import matplotlib.pyplot as plt

import yaml
from commonroad.common.file_reader import CommonRoadFileReader
from commonroad.planning.planning_problem import PlanningProblemSet
from commonroad.scenario.scenario import Scenario

from route_planner.route_planner import RoutePlanner


def load_config_file(filename) -> dict:
    # open config file
    with open(filename, 'r') as stream:
        try:
            configs = yaml.load(stream, Loader=yaml.BaseLoader)
        except yaml.YAMLError as exc:
            print(exc)
    return configs


def release_logger(logger):
    """
    Releases the logger
    :param logger: the logger to be released
    """
    handlers = logger.handlers[:]
    for handler in handlers:
        handler.close()
        logger.removeHandler(handler)


def initialize_logger(logger_name, config_file) -> logging.Logger:
    logger = logging.getLogger(logger_name)
    release_logger(logger)
    logger.setLevel(logging.DEBUG)
    # formatter = logging.Formatter('%(asctime)s - %(name)s - %(levelname)s - %(message)s')
    formatter = logging.Formatter('%(asctime)s\t%(name)s\t%(levelname)s\t%(message)s')

    # create console handler
    console_handler = logging.StreamHandler()
    # set the level of logging to console
    console_handler.setLevel(logging.DEBUG)
    console_handler.setFormatter(formatter)
    logger.addHandler(console_handler)

    if config_file['log_to_file'] == 'True':
        log_file_dir = config_file['log_file_dir']
        date_time_string = ''
        if config_file['add_timestamp_to_log_file'] == 'True':
            now = datetime.now()  # current date and time
            date_time_string = now.strftime("_%Y_%m_%d_%H-%M-%S")

        # if directory not exists create it
        os.makedirs(log_file_dir, exist_ok=True)

        log_file_path = os.path.join(log_file_dir, config_file['log_file_name'] + date_time_string + ".log")
        file_handler = logging.FileHandler(log_file_path)
        # set the level of logging to file
        file_handler.setLevel(logging.DEBUG)
        file_handler.setFormatter(formatter)
        logger.addHandler(file_handler)

    return logger


def get_existing_scenarios(root_dir) -> dict:
    scenarios = dict()
    for path, directories, files in os.walk(root_dir):
        for scenario in fnmatch.filter(files, "*.xml"):
            # res = os.path.normpath(path).split(os.path.sep)
            # rel_path_to_scenario_from_root = os.path.join(*res[1:])
            rel_path_to_scenario_from_root = path
            scenario_name = scenario[:-4]  # chop the '.xml' extension
            scenarios[scenario_name] = rel_path_to_scenario_from_root

    return scenarios


def get_existing_pickle_scenarios(root_dir) -> dict:
    scenarios = dict()
    for path, directories, files in os.walk(root_dir):
        for scenario in fnmatch.filter(files, "*.pickle"):
            # res = os.path.normpath(path).split(os.path.sep)
            # rel_path_to_scenario_from_root = os.path.join(*res[1:])
            rel_path_to_scenario_from_root = path
            scenario_name = scenario[:-7]  # chop the '.xml' extension
            scenarios[scenario_name] = rel_path_to_scenario_from_root

    return scenarios


def get_existing_scenario_ids(root_dir) -> list:
    existing_scenarios = get_existing_scenarios(root_dir)
    scenario_ids = list(existing_scenarios.keys())
    return scenario_ids


def get_existing_pickle_scenario_ids(root_dir) -> list:
    existing_scenarios = get_existing_pickle_scenarios(root_dir)
    scenario_ids = list(existing_scenarios.keys())
    return scenario_ids


def load_scenario(root_dir, scenario_id) -> Tuple[Scenario, PlanningProblemSet]:
    # TODO check if path_to_scenario valid
    path_to_scenario = get_existing_scenarios(root_dir)[scenario_id]
    # scenario_path = os.path.join(root_dir, path_to_scenario, scenario_id + '.xml')
    scenario_path = os.path.join(path_to_scenario, scenario_id + '.xml')
    # open and read in scenario and planning problem set
    scenario, planning_problem_set = CommonRoadFileReader(scenario_path).open()

    return scenario, planning_problem_set


def load_pickle_scenario(root_dir, scenario_id) -> Tuple[Scenario, PlanningProblemSet]:
    path_to_scenario = get_existing_pickle_scenarios(root_dir)[scenario_id]
    scenario_path = os.path.join(path_to_scenario, scenario_id + '.pickle')
    # load scenario and planning problem set
    with open(scenario_path, "rb") as pickle_scenario:
        time1 = time.perf_counter()
        loaded_scenario_tuple = pickle.load(pickle_scenario)
        pickle_load_time = (time.perf_counter() - time1) * 1000
        print("Loading scenario [{:<20}] took\t{:10.4f}\tms".format(scenario_id, pickle_load_time))

    return loaded_scenario_tuple


def load_scenarios(root_dir, scenario_ids):
    for scenario_id in scenario_ids:
        yield load_scenario(root_dir, scenario_id)


def load_pickle_scenarios(root_dir, scenario_ids):
    for scenario_id in scenario_ids:
        yield load_pickle_scenario(root_dir, scenario_id)


def execute_search(scenario, planning_problem, backend="priority_queue"):
    route_planner = RoutePlanner(scenario, planning_problem, backend=backend)

    return route_planner.plan_routes()


def get_last_time_step_in_scenario(scenario: Scenario):
    last_time_step = 0
    for dy_ob in scenario.dynamic_obstacles:
        time_step = len(dy_ob.prediction.occupancy_set)
        if time_step > last_time_step:
            last_time_step = time_step

    return last_time_step


def dim(a):
    if not type(a) == list:
        return []
    return [len(a)] + dim(a[0])
