# Copyright 2024 bj.guerreiro@fct.unl.pt

import sys

import rclpy
from rclpy.executors import ExternalShutdownException
from rclpy.node import Node
from std_msgs.msg import String
# from crazyflie_interfaces.msg import FullState
# from geometry_msgs.msg import Point, Pose
import numpy as np
import rowan


class CybaerTalker(Node):
    """
    A node with a single publisher.

    This class creates a node which regularly publishes messages on a topic. Creating a node by
    inheriting from Node is recommended because it allows it to be imported and used by
    other scripts.
    """

    def __init__(self):
        # Calls Node.__init__('cybaer_talker')
        super().__init__('cybaer_talker')
        cfname = 'cf4tmp'
        prefix = '/' + cfname
        self.prefix = prefix
        self.i = 0
        self.pub = self.create_publisher(String, 'chatter', 10)
        # self.cmdFullStatePublisher = self.create_publisher(
        #     FullState, prefix + '/cmd_full_state', 1)
        # self.cmdFullStateMsg = FullState()
        # self.cmdFullStateMsg.header.frame_id = '/world'

        # Create a timer that calls a callback every second. A timer is recommended for executing
        # periodic tasks because it does not block the main thread while it's waiting. This allows
        # an executor to do other work when mutliple nodes are run in the same process.
        timer_period = 0.5 # seconds
        self.timer = self.create_timer(timer_period, self.timer_callback)

    def timer_callback(self):
        msg = String()
        msg.data = 'Hello World: {0}'.format(self.i)
        self.i += 1
        self.get_logger().info('Publishing: "{0}"'.format(msg.data))
        self.pub.publish(msg)

        # # do some control...
        # pos = np.array([np.nan, np.nan, np.nan])
        # vel = np.array([np.nan, np.nan, np.nan])
        # acc = np.array([1.0, 2.0, 3.0])
        # yaw = 0.1
        # omega = np.array([0.1, 0.3, 0.7])

        # # publish cmdFullState
        # self.cmdFullState(self, pos, vel, acc, yaw, omega)
        # self.get_logger().info('Publishing cmdFullState.')

    # def cmdFullState(self, pos, vel, acc, yaw, omega):
    #     """
    #     Send a streaming full-state controller setpoint command.

    #     The full-state setpoint is most useful for aggressive maneuvers where
    #     feedforward inputs for acceleration and angular velocity are critical
    #     to obtaining good tracking performance. Full-state setpoints can be
    #     obtained from any trajectory parameterization that is at least three
    #     times differentiable, e.g. piecewise polynomials.

    #     Sending a streaming setpoint of any type will force a change from
    #     high-level to low-level command mode. Currently, there is no mechanism
    #     to change back, but it is a high-priority feature to implement.
    #     This means it is not possible to use e.g. :meth:`land()` or
    #     :meth:`goTo()` after a streaming setpoint has been sent.

    #     Args:
    #         pos (array-like of float[3]): Position. Meters.
    #         vel (array-like of float[3]): Velocity. Meters / second.
    #         acc (array-like of float[3]): Acceleration. Meters / second^2.
    #         yaw (float): Yaw angle. Radians.
    #         omega (array-like of float[3]): Angular velocity in body frame.
    #             Radians / sec.

    #     """
    #     # self.cmdFullStateMsg.header.stamp = self.node.get_clock().now().to_msg()
    #     self.cmdFullStateMsg.pose.position.x = pos[0]
    #     self.cmdFullStateMsg.pose.position.y = pos[1]
    #     self.cmdFullStateMsg.pose.position.z = pos[2]
    #     self.cmdFullStateMsg.twist.linear.x = vel[0]
    #     self.cmdFullStateMsg.twist.linear.y = vel[1]
    #     self.cmdFullStateMsg.twist.linear.z = vel[2]
    #     self.cmdFullStateMsg.acc.x = acc[0]
    #     self.cmdFullStateMsg.acc.y = acc[1]
    #     self.cmdFullStateMsg.acc.z = acc[2]
    #     q = rowan.from_euler(0, 0, yaw)
    #     self.cmdFullStateMsg.pose.orientation.w = q[0]
    #     self.cmdFullStateMsg.pose.orientation.x = q[1]
    #     self.cmdFullStateMsg.pose.orientation.y = q[2]
    #     self.cmdFullStateMsg.pose.orientation.z = q[3]
    #     self.cmdFullStateMsg.twist.angular.x = omega[0]
    #     self.cmdFullStateMsg.twist.angular.y = omega[1]
    #     self.cmdFullStateMsg.twist.angular.z = omega[2]
    #     self.cmdFullStatePublisher.publish(self.cmdFullStateMsg)


def main(args=None):
    """
    Run a Talker node standalone.

    This function is called directly when using an entrypoint. Entrypoints are configured in
    setup.py. This along with the script installation in setup.cfg allows a talker node to be run
    with the command `ros2 run examples_rclpy_executors talker`.

    :param args: Arguments passed in from the command line.
    """
    # Run standalone
    rclpy.init(args=args)
    try:
        cybaer_talker = CybaerTalker()
        rclpy.spin(cybaer_talker)
    except KeyboardInterrupt:
        pass
    except ExternalShutdownException:
        sys.exit(1)
    finally:
        rclpy.try_shutdown()
        cybaer_talker.destroy_node()


if __name__ == '__main__':
    # Runs a talker node when this script is run directly (not through an entrypoint)
    main()