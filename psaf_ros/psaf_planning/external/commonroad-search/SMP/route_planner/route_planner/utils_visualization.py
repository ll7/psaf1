from typing import List

import matplotlib.pyplot as plt
from commonroad.geometry.shape import Rectangle
from commonroad.planning.planning_problem import PlanningProblem
from commonroad.prediction.prediction import TrajectoryPrediction
from commonroad.scenario.scenario import Scenario
from commonroad.scenario.trajectory import Trajectory, State
from commonroad.visualization.draw_dispatch_cr import draw_object

from SMP.route_planner.route_planner.route import Route


def draw_state(state: State, fig_num=None, color='red'):
    if fig_num is not None:
        plt.figure(fig_num)

    ego_shape = Rectangle(length=5, width=2)
    if hasattr(state, 'time_step'):
        initial_time_step = int(state.time_step)
    else:
        state.time_step = 0
        initial_time_step = 0
    trajectory = Trajectory(initial_time_step=initial_time_step, state_list=[state])
    prediction = TrajectoryPrediction(trajectory=trajectory, shape=ego_shape)
    draw_object(prediction.occupancy_at_time_step(initial_time_step).shape,
                draw_params={'shape': {'facecolor': f'{color}'}})


def draw_scenario(scenario: Scenario, planning_problem: PlanningProblem, initial_state_color='red', plot_limits=None):
    handles = {}  # collects handles of obstacle patches, plotted by matplotlib

    # plot the lanelet network and the planning problem
    if plot_limits:
        draw_object(scenario, handles=handles, plot_limits=plot_limits)
    else:
        draw_object(scenario, handles=handles)

    draw_object(planning_problem, handles=handles)

    draw_state(planning_problem.initial_state, color=initial_state_color)

    return handles


def plot_found_routes(scenario: Scenario, planning_problem: PlanningProblem, found_routes: List[List]):
    for ctn, route in enumerate(found_routes):
        handles = draw_scenario(scenario, planning_problem)

        # draw ego vehicle - with a collision object - uses commonroad_cc.visualizer
        try:
            draw_state(planning_problem.initial_state)
        except AssertionError as error:
            print(error)

        for route_lanelet_id in route:
            lanelet = scenario.lanelet_network.find_lanelet_by_id(route_lanelet_id)
            draw_object(lanelet, handles=handles, draw_params={'lanelet': {
                'center_bound_color': '#3232ff',  # color of the found route
                'draw_stop_line': False,
                'stop_line_color': '#ff0000',
                'draw_line_markings': True,
                'draw_left_bound': False,
                'draw_right_bound': False,
                'draw_center_bound': True,
                'draw_border_vertices': False,
                'draw_start_and_direction': True,
                'show_label': False,
                'draw_linewidth': 2,
                'fill_lanelet': False,
                'facecolor': '#c7c7c7',
                'zorder': 30  # put it higher in the plot, to make it visible
            }})

            # TODO: the goal region now is covering the lanelet arrows, solution plot a simple blue line on it
            plt.plot(lanelet.center_vertices[:, 0], lanelet.center_vertices[:, 1], "b", zorder=30, scalex=False,
                     scaley=False)

        plt.show()


def plot_route_environment(scenario: Scenario, planning_problem: PlanningProblem, route_environment: List[List]):
    handles = draw_scenario(scenario, planning_problem)

    for ctn, section in enumerate(route_environment):

        for route_lanelet_id in section:
            lanelet = scenario.lanelet_network.find_lanelet_by_id(route_lanelet_id)
            draw_object(lanelet, handles=handles, draw_params={'lanelet': {
                # 'left_bound_color': '#0de309',
                # 'right_bound_color': '#0de309',
                # 'center_bound_color': '#0de309',
                'unique_colors': False,  # colorizes center_vertices and labels of each lanelet differently
                'draw_stop_line': True,
                'stop_line_color': '#ffffff',
                'draw_line_markings': True,
                'draw_left_bound': True,
                'draw_right_bound': True,
                'draw_center_bound': False,
                'draw_border_vertices': False,
                'draw_start_and_direction': True,
                'show_label': False,
                'draw_linewidth': 0.5,
                'fill_lanelet': True,
                'facecolor': '#0de309'
            }})

            # todo: the goal region now is covering the lanelet arrows, solution plot a simple blue line on it
            plt.plot(lanelet.center_vertices[:, 0], lanelet.center_vertices[:, 1], "b", zorder=30, scalex=False,
                     scaley=False)

    plt.show()


def draw_navigator(navigator):
    handles = draw_scenario(navigator.scenario, navigator.planning_problem, initial_state_color='#ed9d98')

    for ctn, section in enumerate(navigator.list_sections):

        for route_lanelet_id in section:
            lanelet = navigator.scenario.lanelet_network.find_lanelet_by_id(route_lanelet_id)
            draw_object(lanelet, handles=handles, draw_params={'lanelet': {
                # 'left_bound_color': '#0de309',
                # 'right_bound_color': '#0de309',
                # 'center_bound_color': '#0de309',
                'unique_colors': False,  # colorizes center_vertices and labels of each lanelet differently
                'draw_stop_line': False,
                'stop_line_color': '#ffffff',
                'draw_line_markings': False,
                'draw_left_bound': False,
                'draw_right_bound': False,
                'draw_center_bound': False,
                'draw_border_vertices': False,
                'draw_start_and_direction': False,
                'show_label': False,
                'draw_linewidth': 0.5,
                'fill_lanelet': True,
                'facecolor': '#00b8cc'
            }})

    for route_merged_lanelet in navigator.merged_route_lanelets:
        draw_object(route_merged_lanelet, handles=handles, draw_params={'lanelet': {
            # 'left_bound_color': '#0de309',
            # 'right_bound_color': '#0de309',
            # 'center_bound_color': '#0de309',
            'unique_colors': False,  # colorizes center_vertices and labels of each lanelet differently
            'draw_stop_line': True,
            'stop_line_color': '#ffffff',
            'draw_line_markings': True,
            'draw_left_bound': True,
            'draw_right_bound': True,
            'draw_center_bound': False,
            'draw_border_vertices': False,
            'draw_start_and_direction': True,
            'show_label': False,
            'draw_linewidth': 0.5,
            'fill_lanelet': True,
            'facecolor': '#128c01'
        }})


def draw_route(route: Route, draw_route_lanelets=False, draw_reference_path=False, plot_limits=None):
    handles = draw_scenario(route.scenario, route.planning_problem, initial_state_color='#ed9d98',
                            plot_limits=plot_limits)

    if draw_route_lanelets:
        for id_lanelet in route.list_ids_lanelets:
            lanelet = route.scenario.lanelet_network.find_lanelet_by_id(id_lanelet)
            draw_object(lanelet, handles=handles, draw_params={'lanelet': {
                'unique_colors': False,  # colorizes center_vertices and labels of each lanelet differently
                'draw_stop_line': False,
                'stop_line_color': '#ffffff',
                'draw_line_markings': True,
                'draw_left_bound': False,
                'draw_right_bound': False,
                'draw_center_bound': True,
                'draw_border_vertices': False,
                'draw_start_and_direction': True,
                'show_label': False,
                'draw_linewidth': 1,
                'fill_lanelet': True,
                'facecolor': '#00b8cc',  # color for filling
                'zorder': 30,  # put it higher in the plot, to make it visible
                'center_bound_color': '#3232ff',  # color of the found route with arrow
            }})

    if draw_reference_path:
        plt.plot(route.reference_path[:, 0], route.reference_path[:, 1], '-m', linewidth=3.5, zorder=31)


def get_plot_limits_from_routes(route, border=15):
    x_min_values = list()
    x_max_values = list()
    y_min_values = list()
    y_max_values = list()
    for route_lanelet_id in route.list_ids_lanelets:
        lanelet = route.scenario.lanelet_network.find_lanelet_by_id(route_lanelet_id)
        x_min_values.append(lanelet.center_vertices[:, 0].min())
        x_max_values.append(lanelet.center_vertices[:, 0].max())
        y_min_values.append(lanelet.center_vertices[:, 1].min())
        y_max_values.append(lanelet.center_vertices[:, 1].max())

    plot_limits = [min(x_min_values) - border, max(x_max_values) + border,
                   min(y_min_values) - border, max(y_max_values) + border]
    return plot_limits


def get_plot_limits_from_reference_path(route, border=10):
    x_min = min(route.reference_path[:, 0])
    x_max = max(route.reference_path[:, 0])
    y_min = min(route.reference_path[:, 1])
    y_max = max(route.reference_path[:, 1])

    plot_limits = [x_min - border, x_max + border, y_min - border, y_max + border]
    return plot_limits
