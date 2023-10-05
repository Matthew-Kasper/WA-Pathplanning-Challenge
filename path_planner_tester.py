import argparse

import yaml

import map_info
from map_info import MapInfo
from path_planner import PathPlanner

class PathPlannerTester:
    @staticmethod
    def generate_full_path_test():
        test_nodes = [(0, 0), (0, 3), (3, 3), (5, 5)]

        path = PathPlanner.generate_full_path(test_nodes)

        expected_path = [(0, 0), (0, 1), (0, 2), (0, 3), (1, 3), (2, 3), (3, 3), (4, 4), (5, 5)]

        for i in range(0, len(path)):
            # Check to make sure generated path is correct
            if path[i] != expected_path[i]:
                print("Error: wrong path step found.")
                print("Expected: " + str(expected_path[i]))
                print("Actual: " + str(path[i]))
                return False

        # Path matches expected path
        return True

    @staticmethod
    def evaluate_path_segment_test():
        path_planner = PathPlanner(PathPlannerTester.generate_map_info(), [map_info.Destination(0, 20, 0, "Test")])

        # Risk of 4 times gain of 7 plus distance squared
        expected_cost = (4 * 5) + 3
        real_cost = path_planner.evaluate_path_segment((38, 30), (42, 30), (45, 30))

        if real_cost != expected_cost:
            print("Error: evaluate_path_segment did not evaluate correct cost.")
            print("Expected: " + str(expected_cost))
            print("Actual: " + str(real_cost))
            return False

        # All tests passed
        return True

    @staticmethod
    def generate_map_info():
        parser = argparse.ArgumentParser()
        parser.add_argument("--map-info-path", required=False, type=str, default="map_info.yaml")
        args = parser.parse_args()

        with open(args.map_info_path) as file:
            config_dict = yaml.load(file, Loader=yaml.FullLoader)

        return MapInfo(config_dict)


    @staticmethod
    def run_tests():
        # Run all of the tests
        generate_full_path_results = PathPlannerTester.generate_full_path_test()
        evaluate_path_segment_results = PathPlannerTester.evaluate_path_segment_test()

        # Print Results
        if generate_full_path_results:
            print("Generate Full Path Test: PASSED")
        else:
            print("Generate Full Path Test: FAILED")

        if evaluate_path_segment_results:
            print("Evaluate Path Segment Test: PASSED")
        else:
            print("Evaluate Path Segment Test: FAILED")

if __name__ == "__main__":
    PathPlannerTester.run_tests()
