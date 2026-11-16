"""Normalize the vendor driver's odom-axis twist for Nav2 body-frame feedback.

The current driver differentiates its world-frame position and puts that vector
in Odometry.twist without rotating it to child_frame_id.  This adapter changes
only twist and its covariance; the original pose, frames and stamp are retained.
It never publishes TF.
"""
import copy
import math
import time

from nav_msgs.msg import Odometry
import rclpy
from rclpy.node import Node
from rclpy.qos import qos_profile_sensor_data

from .geometry import finite, quaternion, rotate, xyz, xyzw


def body_frame_twist(rotation, linear, angular, covariance, input_twist_frame='odom'):
    """Return twist in body axes and C_body = diag(R^-1, R^-1) C J^T.

    ``rotation`` maps the body frame into odom.  Both velocity vectors refer to
    the same body origin, so this is an axes change without a lever-arm term.
    ``body`` explicitly opts out of conversion for a corrected driver/simulator.
    """
    q = quaternion(rotation)
    linear, angular = finite(linear), finite(angular)
    if len(linear) != 3 or len(angular) != 3:
        raise ValueError('twist vectors must each have three entries')
    cov = finite(covariance)
    if len(cov) != 36 or any(cov[i * 7] < 0. for i in range(6)):
        raise ValueError('invalid twist covariance')
    if input_twist_frame == 'body':
        return linear, angular, list(cov)
    if input_twist_frame != 'odom':
        raise ValueError('input_twist_frame must be odom or body')

    inverse_rotation = (-q[0], -q[1], -q[2], q[3])
    axes = [rotate(inverse_rotation, axis)
            for axis in ((1., 0., 0.), (0., 1., 0.), (0., 0., 1.))]
    jacobian = [[axes[j % 3][i % 3] if i // 3 == j // 3 else 0.
                 for j in range(6)] for i in range(6)]
    intermediate = [[sum(jacobian[i][k] * cov[k * 6 + j] for k in range(6))
                     for j in range(6)] for i in range(6)]
    rotated_covariance = finite(
        sum(intermediate[i][k] * jacobian[j][k] for k in range(6))
        for i in range(6) for j in range(6))
    return (finite(rotate(inverse_rotation, linear)), finite(rotate(inverse_rotation, angular)),
            list(rotated_covariance))


def normalized_odometry(message, input_twist_frame='odom', expected_odom_frame='odom'):
    """Copy a valid wheel sample and normalize its twist without changing time."""
    if message.header.frame_id != expected_odom_frame or not message.child_frame_id:
        raise ValueError('unexpected wheel odometry frame')
    finite(xyz(message.pose.pose.position))
    linear, angular, covariance = body_frame_twist(
        xyzw(message.pose.pose.orientation), xyz(message.twist.twist.linear),
        xyz(message.twist.twist.angular), message.twist.covariance, input_twist_frame)
    result = copy.deepcopy(message)
    result.twist.twist.linear.x, result.twist.twist.linear.y, result.twist.twist.linear.z = linear
    (result.twist.twist.angular.x, result.twist.twist.angular.y,
     result.twist.twist.angular.z) = angular
    result.twist.covariance = covariance
    return result


class WheelOdomAdapter(Node):
    def __init__(self):
        super().__init__('wheel_odom_adapter')
        defaults = {
            'input_topic': '/rby1/odom',
            'output_topic': '/rby1/vslam/wheel_odom',
            'input_twist_frame': 'odom',
            'expected_odom_frame': 'odom',
        }
        for name, value in defaults.items():
            self.declare_parameter(name, value)
        self.parameters = {name: self.get_parameter(name).value for name in defaults}
        if any(not isinstance(value, str) or not value.strip()
               for value in self.parameters.values()):
            raise ValueError('wheel odometry parameters must be nonempty strings')
        if self.parameters['input_twist_frame'] not in ('odom', 'body'):
            raise ValueError('input_twist_frame must be odom or body')
        if (self.resolve_topic_name(self.parameters['input_topic'])
                == self.resolve_topic_name(self.parameters['output_topic'])):
            raise ValueError('input_topic and output_topic must differ')
        self.publisher = self.create_publisher(Odometry, self.parameters['output_topic'], 10)
        self.subscription = self.create_subscription(
            Odometry, self.parameters['input_topic'], self.on_odometry, qos_profile_sensor_data)
        self.last_warning = -math.inf
        self.get_logger().info(
            f'Wheel twist {self.parameters["input_twist_frame"]} -> body: '
            f'{self.parameters["input_topic"]} -> {self.parameters["output_topic"]}')

    def on_odometry(self, message):
        try:
            output = normalized_odometry(
                message, self.parameters['input_twist_frame'],
                self.parameters['expected_odom_frame'])
        except (ValueError, TypeError, OverflowError) as exc:
            now = time.monotonic()
            if now - self.last_warning >= 2.:
                self.get_logger().warning(f'Rejected wheel odometry: {exc}')
                self.last_warning = now
            return
        self.publisher.publish(output)


def main(args=None):
    rclpy.init(args=args)
    node = None
    try:
        node = WheelOdomAdapter()
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        if node is not None:
            node.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()


if __name__ == '__main__':
    main()
