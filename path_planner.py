"""
    This module is your primary workspace. Add whatever helper functions, classes, data structures, imports... etc here.

    We expect most results will utilize more than just dumping code into the plan_paths()
        function, that just serves as a meaningful entry point.

    In order for the rest of the scoring to work, you need to make sure you have correctly
        populated the Destination.path for each result you produce.
"""
import math
import sys
import typing
from queue import PriorityQueue

import numpy as np
from typing import Dict
import structlinks;

import map_info
from map_info import Coordinate, Destination, MapInfo


class PathPlanner:

    def __init__(self, map_info: MapInfo, destinations: typing.List["Destination"]):
        self.map_info: MapInfo = map_info
        self.destinations: typing.List["Destination"] = destinations

    def plan_paths(self):
        """
        This is the function you should re-write. It is expected to mutate the list of
        destinations by calling each Destination's set_path() with the resulting
        path as an argument.

        The default construction shows this format, and should produce 10 invalid paths.
        """
        for site in self.destinations:
            print("Finding path for: " + site.name)

            start_coordinate = self.map_info.start_coord
            end_coordinate = site.coord
            # Create linked list to store path nodes
            nodes = [(start_coordinate[0], start_coordinate[1]), (end_coordinate[0], end_coordinate[1])]

            at_goal = False
            node_number = 0
            while not at_goal:
                node_number = node_number + 1
                least_costly_node = (-1, -1)

                # Search through all possible node locations and compare their costs
                for i in range (0, len(self.map_info.risk_zones)):
                    for j in range(0, len(self.map_info.risk_zones[i])):
                        new_node = (i, j)

                        # Find cost of new node
                        new_node_cost = self.evaluate_path_segment(nodes[node_number - 1], new_node, end_coordinate)
                        # Skip if path is impossible
                        if new_node_cost == -1:
                            continue

                        # Don't add a node that already exists in the list

                        is_in_nodes = False
                        for node in nodes:
                            if new_node != end_coordinate and new_node == node:
                                is_in_nodes = True

                        if is_in_nodes:
                            continue

                        # Set current node as default first node, nothing to compare it against
                        if least_costly_node == (-1, -1):
                            least_costly_node = new_node
                            continue

                        # Find favorability of most favorable known node
                        most_favorable_node_cost = self.evaluate_path_segment(nodes[node_number - 1], least_costly_node,
                                                                              end_coordinate)

                        # If new node is more favorable
                        if new_node_cost < most_favorable_node_cost:
                            least_costly_node = new_node
                # Stop condition
                if least_costly_node == end_coordinate:
                    at_goal = True
                else:
                    # Add node to node list and try again
                    nodes.insert(node_number, least_costly_node)
                    print("Added Node: " + str(least_costly_node))
                    print("Cost: " + str(self.evaluate_path_segment(nodes[node_number - 1], least_costly_node,
                                                                              site.coord)))

            # Convert list of nodes into a path with step sizes of 1
            path_array = PathPlanner.generate_full_path(nodes)

            path_coords = [Coordinate(arr[0], arr[1]) for arr in path_array]
            # Once you have a solution for the site - populate it like this:
            site.set_path(path_coords)

    # Generates a cost of taking a specific route. Lower numbers are more favorable, higher numbers are less favorable.
    # -1 is a special cost value that means that the path is impossible and should never take it
    def evaluate_path_segment(self, starting_node, ending_node, goal_node):
        segment_steps = PathPlanner.generate_full_path([starting_node, ending_node])
        # Keep a counter to increment everytime a step falls within a low risk area
        total_risk = 0

        # How much weight risk has in cost
        RISK_GAIN = .7

        # How much weight segment length has in cost
        LENGTH_GAIN = .2

        # How much weight being close to goal has in cost
        TARGET_GAIN = 1

        for step in segment_steps:
            step_risk = self.map_info.risk_zones[step[0]][step[1]]

            # Automatically quit with -1 if there is a step that contains a keep out zone
            if step_risk == MapInfo.KEEP_OUT_VALUE:
                return -1
            elif step_risk == MapInfo.HIGH_RISK_VALUE:
                total_risk = total_risk + 1

        distance_to_goal = math.sqrt(math.pow(goal_node[0] - ending_node[0], 2) + \
                                    math.pow(goal_node[1] - ending_node[1], 2))

        segment_length = math.sqrt(math.pow(starting_node[0] - ending_node[0], 2) +
                                   math.pow(starting_node[1] - ending_node[1], 2))

        # Cost is a combination of total risk, segment length, and distance to goal
        return int(total_risk * RISK_GAIN) + int(segment_length * LENGTH_GAIN) + int(distance_to_goal * TARGET_GAIN)

    @staticmethod
    # Takes a list of nodes and transforms it into a usable coordinate path. Nodes consists of a list of coordinates
    def generate_full_path(nodes):
        # Initialize path array with starting position
        path_array = [nodes[0]]

        # Iterate through each pair of connected nodes and add their path with step size 1 to path_array
        for i in range(0, len(nodes) - 1):
            # Match the position the line is in with a coordinate in one of the 8 locations around the current location
            current_location = nodes[i]

            # Continue until the current location equals the destination
            while current_location != nodes[i + 1]:
                # We can assume that step is going to travel in steps of 1 towards direction of the node

                to_next_node_vector = (nodes[i + 1][0] - current_location[0], nodes[i + 1][1] - current_location[1])
                next_move_addition_vector = (0, 0)

                # Special case where can't find angle of movement vector
                if (to_next_node_vector[0] == 0):
                    next_move_addition_vector = (0, np.sign(to_next_node_vector[1]))
                else:
                    # Standard case where next position lies on 6 tiles to left and right of current position
                    angle_of_move_vector = math.atan(to_next_node_vector[1] / to_next_node_vector[0])

                    # Find if next position lies of the right or left
                    x_direction = np.sign(to_next_node_vector[0])
                    y_direction = np.sign(to_next_node_vector[1])

                    # Threshold vector value to treat very slight angles in a direction as 0
                    if math.pi / 3 <= angle_of_move_vector:
                        next_move_addition_vector = (0, y_direction)
                    elif math.pi / 6 < angle_of_move_vector < math.pi / 3:
                        next_move_addition_vector = (x_direction, y_direction)
                    elif -math.pi / 6 <= angle_of_move_vector <= math.pi / 6 :
                        next_move_addition_vector = (x_direction, 0)
                    elif -math.pi / 3 < angle_of_move_vector < -math.pi / 6:
                        next_move_addition_vector = (x_direction, y_direction)
                    else:
                        next_move_addition_vector = (0, y_direction)

                # Find x and y coordinates
                x_coordinate = int(current_location[0] + next_move_addition_vector[0])

                # Vertical only movement is applied if slope is undefined because of it being a vertical line
                y_coordinate = int(current_location[1] + next_move_addition_vector[1])

                # Overwrite the current location with the new current location and add to path_array
                current_location = (x_coordinate, y_coordinate)
                path_array.append(current_location)

        return path_array







