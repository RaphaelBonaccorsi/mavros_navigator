#!/usr/bin/env python3
"""
Mavros Navigator Node
---------------------

This module defines a ROS 2 node that controls a drone via MAVROS. It enables the drone
to navigate to specified waypoints by handling offboard mode activation, drone arming,
setpoint publishing, and monitoring the drone’s position for distance feedback to the target.

Classes
-------
MavrosNavigator
    A node that executes drone waypoint movements using ROS 2 actions.
"""

import rclpy
from rclpy.node import Node
from rclpy.action import ActionServer, GoalResponse, CancelResponse
from geometry_msgs.msg import PoseStamped
from mavros_msgs.srv import CommandBool, SetMode
from math import sqrt
from rclpy.qos import QoSProfile, QoSReliabilityPolicy, QoSHistoryPolicy, QoSDurabilityPolicy
from mavros_navigator_interfaces.action import MoveTo
import time


class MavrosNavigator(Node):
    """
    A node that manages drone waypoint navigation using MAVROS and ROS 2 actions.

    Attributes
    ----------
    publisher_ : rclpy.publisher.Publisher
        Publisher for setpoint positions (PoseStamped messages).
    arming_client : rclpy.client.Client
        Client for the CommandBool service to arm the drone.
    set_mode_client : rclpy.client.Client
        Client for the SetMode service to change the flight mode.
    position_subscriber : rclpy.subscription.Subscription
        Subscriber to the drone's current position topic (/mavros/local_position/pose).
    setpoint_timer : rclpy.timer.Timer
        Timer to periodically publish the current setpoint.
    _action_server : rclpy.action.ActionServer
        Action server for handling waypoint movement requests.
    waypoint : PoseStamped
        The current target waypoint for the drone.
    current_position : PoseStamped
        The current position of the drone.
    _timer : rclpy.timer.Timer or None
        Timer used during the execution of a movement action.
    _cancel_requested : bool
        Flag indicating whether a cancellation has been requested.
    """

    def __init__(self):
        """
        Initializes the MavrosNavigator node.

        It sets up publishers, service clients, subscriptions, timers, and the action server.
        It also activates offboard mode and arms the drone.
        """
        super().__init__('mavros_navigator')

        # Publisher and service clients
        self.publisher_ = self.create_publisher(PoseStamped, '/mavros/setpoint_position/local', 10)
        self.arming_client = self.create_client(CommandBool, '/mavros/cmd/arming')
        self.set_mode_client = self.create_client(SetMode, '/mavros/set_mode')

        # Wait for services to become available
        self.get_logger().info('Waiting for arming and set mode services...')
        self.arming_client.wait_for_service(timeout_sec=10.0)
        self.set_mode_client.wait_for_service(timeout_sec=10.0)

        # Initialize variables
        self.waypoint = PoseStamped()
        self.current_position = PoseStamped()
        self._timer = None  # Timer to manage asynchronous action execution
        self._cancel_requested = False

        # Define QoS profile for the position subscription
        qos_profile = QoSProfile(
            reliability=QoSReliabilityPolicy.BEST_EFFORT,
            durability=QoSDurabilityPolicy.VOLATILE,
            history=QoSHistoryPolicy.KEEP_LAST,
            depth=1
        )

        # Subscribe to the drone's current position
        self.position_subscriber = self.create_subscription(
            PoseStamped, '/mavros/local_position/pose', self.position_callback, qos_profile
        )

        # Set up the action server for moving to a waypoint
        self._action_server = ActionServer(
            self,
            MoveTo,
            '/drone/move_to_waypoint',
            self.execute_callback,
            goal_callback=self.goal_callback,
            cancel_callback=self.cancel_callback
        )

        # Timer for publishing setpoints at 20Hz
        self.setpoint_timer = self.create_timer(0.05, self.publish_current_setpoint)
        self.get_logger().info('Started publishing setpoints at 20Hz...')

        # Activate offboard mode and arm the drone
        self.set_offboard_mode()
        time.sleep(2)
        self.arm()

    def goal_callback(self, goal_request):
        """
        Callback to handle incoming goal requests from action clients.

        Parameters
        ----------
        goal_request : MoveTo.Goal
            The goal request containing the target waypoint.

        Returns
        -------
        GoalResponse
            A response indicating that the goal is accepted.
        """
        self.get_logger().info(f'Receiving move request to: {goal_request.destination.pose}')
        self.waypoint = goal_request.destination
        return GoalResponse.ACCEPT

    def cancel_callback(self, goal_handle):
        """
        Callback to handle cancellation requests for an ongoing action.

        Parameters
        ----------
        goal_handle : rclpy.action.GoalHandle
            The handle for the action being canceled.

        Returns
        -------
        CancelResponse
            A response indicating that the cancellation request is accepted.
        """
        self.get_logger().info('Cancelling goal')
        self._cancel_requested = True
        return CancelResponse.ACCEPT

    async def execute_callback(self, goal_handle):
        """
        Executes the movement action to navigate the drone to the specified waypoint.

        This function periodically publishes the setpoint, checks for waypoint
        reachability or cancellation, and provides feedback on the distance to the waypoint.

        Parameters
        ----------
        goal_handle : rclpy.action.GoalHandle
            The handle for the action being executed.

        Returns
        -------
        MoveTo.Result
            The result of the movement action indicating success or failure.
        """
        self.get_logger().info('Executing movement action')
        feedback_msg = MoveTo.Feedback()
        result = MoveTo.Result()
        self._cancel_requested = False 

        goal_future = rclpy.task.Future()

        def timer_callback():
            nonlocal goal_future

            # Update feedback with the current distance to the waypoint
            feedback_msg.distance = float(self.get_distance(self.waypoint))
            goal_handle.publish_feedback(feedback_msg)

            # Check if the target waypoint is reached
            if self.has_reached_waypoint(self.waypoint):
                self._timer.cancel()  # Cancel the timer
                goal_handle.succeed()
                result.success = True
                self.get_logger().info('Movement completed successfully')
                goal_future.set_result(result)
            # Check if cancellation has been requested
            elif self._cancel_requested:
                self._timer.cancel()  # Cancel the timer
                goal_handle.canceled()
                result.success = False
                self.get_logger().info('Action canceled')
                goal_future.set_result(result)
            # Otherwise, publish the current setpoint again
            else:
                self.publish_current_setpoint()

        # Configure the timer to execute the callback periodically at 20Hz
        self._timer = self.create_timer(0.05, timer_callback)

        # Wait until the goal is either reached or canceled
        await goal_future

        return result

    def position_callback(self, msg):
        """
        Callback to update the drone's current position.

        Parameters
        ----------
        msg : PoseStamped
            The message containing the drone's current position.
        """
        self.current_position = msg

    def get_current_position(self):
        """
        Retrieves the drone's current position.

        Returns
        -------
        geometry_msgs.msg.Point
            The current position of the drone.
        """
        return self.current_position.pose.position

    def publish_current_setpoint(self):
        """
        Publishes the current setpoint for the drone.

        The setpoint is based on the target waypoint's position, with a fixed orientation.
        """
        msg = PoseStamped()
        waypoint = self.waypoint

        msg.pose.position.x = waypoint.pose.position.x
        msg.pose.position.y = waypoint.pose.position.y
        msg.pose.position.z = waypoint.pose.position.z
        msg.pose.orientation.w = 1.0

        self.publisher_.publish(msg)

    def set_offboard_mode(self):
        """
        Sets the drone's flight mode to OFFBOARD.

        It sends a service request to change the flight mode, and logs the outcome.
        """
        self.get_logger().info('Setting mode to OFFBOARD')
        set_mode_request = SetMode.Request()
        set_mode_request.custom_mode = 'OFFBOARD'
        set_mode_future = self.set_mode_client.call_async(set_mode_request)
        rclpy.spin_until_future_complete(self, set_mode_future)

        if set_mode_future.result() is not None and set_mode_future.result().mode_sent:
            self.get_logger().info('Offboard mode set successfully')
        else:
            self.get_logger().error('Failed to set Offboard mode')

    def arm(self):
        """
        Arms the drone by sending a command via the CommandBool service.

        Logs whether the arming command was successful.
        """
        self.get_logger().info('Arming the drone...')
        arm_request = CommandBool.Request()
        arm_request.value = True
        arm_future = self.arming_client.call_async(arm_request)
        rclpy.spin_until_future_complete(self, arm_future)

        if arm_future.result() is not None and arm_future.result().success:
            self.get_logger().info('Drone armed successfully')
        else:
            self.get_logger().error('Failed to arm the drone')

    def has_reached_waypoint(self, waypoint, threshold=1.0):
        """
        Checks if the drone has reached the specified waypoint.

        Parameters
        ----------
        waypoint : PoseStamped
            The target waypoint.
        threshold : float, optional
            The distance threshold to consider the waypoint reached (default is 1.0).

        Returns
        -------
        bool
            True if the drone is within the threshold distance to the waypoint, False otherwise.
        """
        distance = self.get_distance(waypoint)
        return distance < threshold

    def get_distance(self, waypoint):
        """
        Calculates the Euclidean distance between the drone's current position and a target waypoint.

        Parameters
        ----------
        waypoint : PoseStamped
            The target waypoint.

        Returns
        -------
        float
            The calculated distance to the waypoint.
        """
        current_position = self.get_current_position()
        distance = sqrt(
            (waypoint.pose.position.x - current_position.x) ** 2 +
            (waypoint.pose.position.y - current_position.y) ** 2 +
            (waypoint.pose.position.z - current_position.z) ** 2
        )
        return distance


def main(args=None):
    """
    Entry point for the Mavros Navigator node.

    Parameters
    ----------
    args : list, optional
        Command-line arguments passed to the node (default is None).

    """
    rclpy.init(args=args)
    node = MavrosNavigator()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()


if __name__ == '__main__':
    main()
