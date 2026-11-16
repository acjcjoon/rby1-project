"""Apply a spatial median filter to the rectified D435i stereo images."""

import cv2
from cv_bridge import CvBridge, CvBridgeError
import rclpy
from rclpy.node import Node
from rclpy.qos import qos_profile_sensor_data
from sensor_msgs.msg import Image


def validate_kernel_size(value):
    """Return a small, odd OpenCV median kernel size."""
    kernel_size = int(value)
    if kernel_size < 3 or kernel_size > 9 or kernel_size % 2 == 0:
        raise ValueError('kernel_size must be an odd integer from 3 through 9')
    return kernel_size


class StereoMedianFilter(Node):
    def __init__(self):
        super().__init__('stereo_median_filter')
        defaults = {
            'kernel_size': 3,
            'left_input_topic': '/d435/d435/infra1/image_rect_raw',
            'right_input_topic': '/d435/d435/infra2/image_rect_raw',
            'left_output_topic': '/rby1/vslam/median/infra1/image_rect_raw',
            'right_output_topic': '/rby1/vslam/median/infra2/image_rect_raw',
        }
        for name, value in defaults.items():
            self.declare_parameter(name, value)
        parameters = {
            name: self.get_parameter(name).value for name in defaults
        }
        self.kernel_size = validate_kernel_size(parameters['kernel_size'])
        self.bridge = CvBridge()
        self.failures = {'left': 0, 'right': 0}

        for side in ('left', 'right'):
            input_topic = parameters[f'{side}_input_topic']
            output_topic = parameters[f'{side}_output_topic']
            if not input_topic or not output_topic or input_topic == output_topic:
                raise ValueError(
                    f'{side} input/output topics must be non-empty and different')

        self.left_publisher = self.create_publisher(
            Image, parameters['left_output_topic'], qos_profile_sensor_data)
        self.right_publisher = self.create_publisher(
            Image, parameters['right_output_topic'], qos_profile_sensor_data)
        self.left_subscription = self.create_subscription(
            Image, parameters['left_input_topic'],
            lambda message: self._filter(message, self.left_publisher, 'left'),
            qos_profile_sensor_data)
        self.right_subscription = self.create_subscription(
            Image, parameters['right_input_topic'],
            lambda message: self._filter(message, self.right_publisher, 'right'),
            qos_profile_sensor_data)

        self.get_logger().info(
            f'Stereo median filter enabled: {self.kernel_size}x{self.kernel_size}; '
            f"{parameters['left_input_topic']} -> {parameters['left_output_topic']}, "
            f"{parameters['right_input_topic']} -> {parameters['right_output_topic']}")

    def _filter(self, message, publisher, side):
        try:
            image = self.bridge.imgmsg_to_cv2(message, desired_encoding='mono8')
            filtered = cv2.medianBlur(image, self.kernel_size)
            output = self.bridge.cv2_to_imgmsg(filtered, encoding='mono8')
            # Filtering does not alter camera geometry or acquisition time.
            output.header = message.header
            publisher.publish(output)
        except (CvBridgeError, cv2.error, TypeError, ValueError) as error:
            self.failures[side] += 1
            count = self.failures[side]
            if count == 1 or count % 30 == 0:
                self.get_logger().error(
                    f'{side} median filtering failed ({count} frames): {error}')


def main(args=None):
    rclpy.init(args=args)
    node = None
    try:
        node = StereoMedianFilter()
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
