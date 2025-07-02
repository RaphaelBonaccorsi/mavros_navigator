#!/usr/bin/env python3

import rclpy
from rclpy.node import Node
from geometry_msgs.msg import PoseStamped
from ament_index_python.packages import get_package_share_directory
from rclpy.action import ActionClient
from mavros_navigator_interfaces.action import MoveTo


class Map:
    """
    A class to manage a set of waypoints loaded from files.

    Attributes
    ----------
    waypoints : list of PoseStamped
        A list of waypoints as PoseStamped messages with a fixed altitude.
    """

    def __init__(self):
        """
        Initializes the Map object.

        Loads waypoints from pre-defined text files containing coordinate data.
        """
        self.waypoints = []
        self.load_waypoints_from_files()

    def read_coordinates(self, file_path):
        """
        Reads a file with values and returns a list of floats.

        Parameters
        ----------
        file_path : str
            The path to the file containing coordinate values.

        Returns
        -------
        list of float
            A list of coordinate values read from the file.
        """
        with open(file_path, 'r') as file:
            return [float(coord) for coord in file.read().strip().split("\n")]

    def load_waypoints_from_files(self):
        """
        Loads waypoints from the x_coords.txt and y_coords.txt files with a constant altitude.

        The method reads x and y coordinates from respective files located in the package's
        data directory. A fixed altitude (10.0) and default orientation (w=1.0) are assigned to
        each waypoint. Raises a ValueError if the files contain a different number of coordinates.
        """
        package_share_dir = get_package_share_directory('mavros_navigator')
        x_file_path = package_share_dir + "/data/x_coords.txt"
        y_file_path = package_share_dir + "/data/y_coords.txt"
        
        x_coords = self.read_coordinates(x_file_path)
        y_coords = self.read_coordinates(y_file_path)
        
        if len(x_coords) != len(y_coords):
            raise ValueError("The coordinates files should have the same length.")

        # Convert coordinates to a list of PoseStamped waypoints with constant altitude
        self.waypoints = []
        for x, y in zip(x_coords, y_coords):
            pose_stamped = PoseStamped()
            pose_stamped.pose.position.x = x
            pose_stamped.pose.position.y = y
            pose_stamped.pose.position.z = 10.0  # Default altitude
            pose_stamped.pose.orientation.w = 1.0  # Default orientation
            self.waypoints.append(pose_stamped)


class PathPlanner(Node):
    """
    A ROS 2 node for planning and sending waypoints to a drone.

    This node uses an action client to send movement goals (waypoints) to the drone.
    It loads waypoints from files and publishes them sequentially.

    Attributes
    ----------
    map : Map
        An instance of the Map class that stores waypoints.
    action_client : ActionClient
        An action client for sending MoveTo goals.
    current_waypoint_index : int
        Index of the current waypoint in the list of waypoints.
    """

    def __init__(self):
        """
        Initializes the PathPlanner node.

        It creates an instance of the Map, sets up the action client for MoveTo goals, and sends
        the first waypoint if available.
        """
        super().__init__('path_planner')

        # Initialize Map object for handling waypoints
        self.map = Map()

        self.action_client = ActionClient(self, MoveTo, '/drone/move_to_waypoint')
        self.current_waypoint_index = 0

        self.action_client.wait_for_server()

        self.publish_next_waypoint()

    def publish_next_waypoint(self):
        """
        Publishes the next waypoint to the drone using the action client.

        If there are remaining waypoints, this method sends the next MoveTo goal.
        Otherwise, it logs that all waypoints have been published.
        """
        if self.current_waypoint_index < len(self.map.waypoints):
            goal_msg = MoveTo.Goal()
            goal_msg.destination = self.map.waypoints[self.current_waypoint_index]
            
            send_goal_future = self.action_client.send_goal_async(
                goal_msg, feedback_callback=self.feedback_callback
            )
            send_goal_future.add_done_callback(self.goal_response_callback)
        else:
            self.get_logger().info("All waypoints have been published.")

    def feedback_callback(self, feedback_msg):
        """
        Receives feedback from the action server.

        Parameters
        ----------
        feedback_msg : MoveTo.FeedbackMessage
            The feedback message containing data such as the distance to the waypoint.
        """
        feedback = feedback_msg.feedback
        # Process feedback if necessary (currently, no additional action is taken).

    def goal_response_callback(self, future):
        """
        Handles the response from the action server after sending a goal.

        Parameters
        ----------
        future : Future
            A future object containing the goal handle.
        """
        goal_handle = future.result()
        if not goal_handle.accepted:
            self.get_logger().error('Goal rejected')
            return

        result_future = goal_handle.get_result_async()
        result_future.add_done_callback(self.get_result_callback)

    def get_result_callback(self, future):
        """
        Processes the result of a goal after its execution.

        Parameters
        ----------
        future : Future
            A future object containing the result of the goal.

        Notes
        -----
        If the goal is successful, the waypoint index is incremented and the next waypoint is published.
        Otherwise, an error is logged.
        """
        result = future.result().result
        if result:
            self.current_waypoint_index += 1
            self.publish_next_waypoint()
        else:
            self.get_logger().error('Failed to reach waypoint')


def main(args=None):
    """
    Entry point for the PathPlanner node.

    Initializes the ROS 2 communication, creates the PathPlanner node, and spins until shutdown.

    Parameters
    ----------
    args : list, optional
        Command-line arguments (default is None).
    """
    rclpy.init(args=args)

    # Create the node and spin
    path_planner = PathPlanner()
    rclpy.spin(path_planner)

    # Shutdown the node after completion
    path_planner.destroy_node()
    rclpy.shutdown()


if __name__ == '__main__':
    main()
