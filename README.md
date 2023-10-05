# WA-Pathplanning-Challenge
 Pathplanning solution for the Wisconsin Autonomous path planning challenge.

# Algorithm Overview

The algorithm is broken up into three parts
1. A function that takes two position nodes and generates a sequence of coordinates linking them together with step-size one.
2. A function that scores these step-size 1 paths on the basis of risk and ending distance to target
3. A function that generates a simplification of the route in key position nodes where the segment from node to node has the least possible cost in the set of all possible segments originating from the start of the segment.

# Details

1. For each step, function 1 generates a vector pointing from the current step location to the goal and thresholds the angle of the vector to correspond to one of the eight possible next step locations around the current step. The process continues until the it has reached a step equal to the goal. A list of coordinates for steps from the starting node to the ending node is returned.

2. To balance risk-taking with length, a count of all the risk tiles traversed over and the distance to the goal were used to compile a cost metric. Paths with smaller cost values would eventually be chosen over paths with larger cost values except in the case when a path has a cost of -1 which corresponds to impossible. A cost value of -1 is reached if a keep-out tile is traversed. Rather than checking if a certain complete path was above the maximum length, position to goal is used as a variable in calculating the cost. When position to goal is given much influence, paths tend to take straighter and shorter paths to the goal with risk being less influential. Proper tuning of the risk and target gain values can lead to a good balance of both or special preference over one another.

3. After these sub-algorithms have been developed, searching for paths becomes much easier. Complete paths are broken up into key position nodes form linear segments. The optimal linear segment for the last added node (the current position) is found by determining the cost of every possible segment from this node to a node in every possible position on the grid. The function keeps the node with the lowest cost and adds it to a list of key nodes. The process is repeated with this new node as the starting position of the segment until a node is added that equals the goal.
