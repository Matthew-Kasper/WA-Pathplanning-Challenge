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
            # Create linked list to store path nodes
            nodes = structlinks.DataStructures.LinkedList([self.map_info.start_coord, site.coord])

            at_goal = False
            node_number = 0
            while not at_goal:
                node_number = node_number + 1
                least_costly_node = (-1, -1)
                # Search through all possible node locations and compare their costs
                for i in range (0, len(self.map_info.risk_zones)):
                    for j in range(0, len(self.map_info.risk_zones[i])):
                        new_node = (i, j)

                        # Set current node as default first node, nothing to compare it against
                        if least_costly_node == (-1, -1):
                            least_costly_node = new_node
                            continue

                        # Find cost of new node
                        new_node_cost = PathPlanner.evaluate_path_segment(nodes[node_number - 1],
                                                                                  new_node, site.coord, self.map_info)
                        # Skip if path is impossible
                        if new_node_cost == -1:
                            continue

                        # Find favorability of most favorable known node
                        most_favorable_node_cost = PathPlanner.evaluate_path_segment(
                            nodes[node_number - 1], least_costly_node, site.coord, self.map_info)

                        # If new node is more favorable
                        if new_node_cost < most_favorable_node_cost:
                            least_costly_node = new_node
                # Stop condition
                if least_costly_node == site.coord:
                    at_goal = True
                else:
                    # Add node to node list and try again
                    nodes.insert(node_number, least_costly_node)

            # Convert list of nodes into a path with step sizes of 1
            path_coords = PathPlanner.generate_full_path(nodes)

            # Once you have a solution for the site - populate it like this:
            site.set_path(path_coords)

    @staticmethod
    # Takes a list of nodes and transforms it into a usable coordinate path. Nodes consists of a list of coordinates
    def generate_full_path(nodes):

        # Initialize path array with starting position

        path_array = [nodes[0]]
        # Iterate through each pair of connected nodes and add their path with step size 1 to path_array
        for i in range(0, len(nodes) - 1):

            # We can assume that x is going to travel in steps of 1 towards direction of the node
            # Find a unit direction to travel in for x, -1 is left, 1 is right
            x_direction = math.copysign(1, nodes[i + 1][0] - nodes[i][0])

            # Match the position the line is in with a coordinate in one of the 8 locations around the current location
            current_location = nodes[i]

            # Continue until the current location equals the destination
            while current_location != nodes[i + 1]:
                # Find a slope representation of current position to the ending node
                slope = (nodes[i + 1][1] - current_location[i][1]) / (nodes[i + 1][0] - current_location[i][0])

                # Find x and y coordinates
                x_coordinate = current_location[0] + x_direction
                y_coordinate = int(current_location[1] + (slope * x_direction))

                # Overwrite the current location with the new current location and add to path_array
                current_location = (x_coordinate, y_coordinate)
                path_array.append(current_location)

        return path_array
    @staticmethod
    # Generates a cost of taking a specific route. Lower numbers are more favorable, higher numbers are less favorable.
    # -1 is a special cost value that means that the path is impossible and should never take it
    def evaluate_path_segment(starting_node, ending_node, goal_node, map_info : MapInfo):
        segment_steps = PathPlanner.generate_full_path([starting_node, ending_node])
        # Keep a counter to increment everytime a step falls within a low risk area
        total_risk = 0

        # How much weight risk has in favorability
        RISK_GAIN = 20

        for step in segment_steps:
            step_risk = map_info.risk_zones[step[0]][step[1]]

            # Automatically quit with -1 if there is a step that contains a keep out zone
            if step_risk == MapInfo.HIGH_RISK_VALUE:
                return -1
            elif step_risk == MapInfo.LOW_RISK_VALUE:
                total_risk = total_risk + 1

        distance_to_goal_squared = math.pow(goal_node[0] - ending_node[0], 2) + \
                                   math.pow(goal_node[1] - ending_node[1], 2)

        # Outside of maximum range
        if int(distance_to_goal_squared) > map_info.maximum_range:
            return -1

        # Favorability is a combination of total risk and length
        return (total_risk * RISK_GAIN) + int(distance_to_goal_squared)







