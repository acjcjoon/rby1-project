#!/usr/bin/env python3

import math

import message_filters
import numpy as np
import rclpy
from rclpy.duration import Duration
from rclpy.node import Node
from rclpy.qos import qos_profile_sensor_data
from rclpy.time import Time
from sensor_msgs.msg import LaserScan
from tf2_ros import Buffer, TransformException, TransformListener


class ScanMergerNode(Node):
    """Merge two LaserScan topics into one virtual 360-degree LaserScan."""

    def __init__(self) -> None:
        super().__init__("scan_merger_node")

        # Existing dual_lidar_node.py publishes these two topics.
        self.declare_parameter("scan0_topic", "/scan_front_right")
        self.declare_parameter("scan1_topic", "/scan_rear_left")

        # Publish as /scan so SLAM Toolbox can use it directly.
        self.declare_parameter("output_topic", "/scan")

        # Both scans are transformed into this common coordinate frame.
        # Change this parameter if your robot base frame has another name.
        self.declare_parameter("target_frame", "base_footprint")

        # Full 360-degree output scan.
        self.declare_parameter("output_angle_min", -math.pi)
        self.declare_parameter("output_angle_max", math.pi)

        # 0.0 means: automatically use the finer resolution of the inputs.
        self.declare_parameter("output_angle_increment", 0.0)

        self.declare_parameter("range_min", 0.02)
        self.declare_parameter("range_max", 20.0)

        # The original node gives /scan0 and /scan1 the same timestamp.
        # A small tolerance still makes startup and transport more robust.
        self.declare_parameter("sync_slop", 0.05)
        self.declare_parameter("sync_queue_size", 20)
        self.declare_parameter("tf_timeout", 0.10)

        self.scan0_topic = str(
            self.get_parameter("scan0_topic").value
        )
        self.scan1_topic = str(
            self.get_parameter("scan1_topic").value
        )
        self.output_topic = str(
            self.get_parameter("output_topic").value
        )
        self.target_frame = str(
            self.get_parameter("target_frame").value
        )

        self.output_angle_min = float(
            self.get_parameter("output_angle_min").value
        )
        self.output_angle_max_requested = float(
            self.get_parameter("output_angle_max").value
        )
        self.output_angle_increment_parameter = float(
            self.get_parameter("output_angle_increment").value
        )

        self.output_range_min = float(
            self.get_parameter("range_min").value
        )
        self.output_range_max = float(
            self.get_parameter("range_max").value
        )

        self.sync_slop = float(
            self.get_parameter("sync_slop").value
        )
        self.sync_queue_size = int(
            self.get_parameter("sync_queue_size").value
        )
        self.tf_timeout = float(
            self.get_parameter("tf_timeout").value
        )

        self._validate_parameters()

        self.tf_buffer = Buffer()
        self.tf_listener = TransformListener(
            self.tf_buffer,
            self,
        )

        self.scan_publisher = self.create_publisher(
            LaserScan,
            self.output_topic,
            qos_profile_sensor_data,
        )

        self.scan0_subscriber = message_filters.Subscriber(
            self,
            LaserScan,
            self.scan0_topic,
            qos_profile=qos_profile_sensor_data,
        )

        self.scan1_subscriber = message_filters.Subscriber(
            self,
            LaserScan,
            self.scan1_topic,
            qos_profile=qos_profile_sensor_data,
        )

        self.synchronizer = (
            message_filters.ApproximateTimeSynchronizer(
                [
                    self.scan0_subscriber,
                    self.scan1_subscriber,
                ],
                queue_size=self.sync_queue_size,
                slop=self.sync_slop,
            )
        )
        self.synchronizer.registerCallback(
            self.merge_callback
        )

        self.get_logger().info(
            f"Subscribing to {self.scan0_topic} and "
            f"{self.scan1_topic}"
        )
        self.get_logger().info(
            f"Publishing merged scan to {self.output_topic}"
        )
        self.get_logger().info(
            f"Merged scan target frame: {self.target_frame}"
        )

    def _validate_parameters(self) -> None:
        if not self.target_frame:
            raise ValueError("target_frame must not be empty")

        if (
            self.output_angle_max_requested
            <= self.output_angle_min
        ):
            raise ValueError(
                "output_angle_max must be larger than "
                "output_angle_min"
            )

        output_span = (
            self.output_angle_max_requested
            - self.output_angle_min
        )

        if output_span > 2.0 * math.pi + 1.0e-6:
            raise ValueError(
                "Output angular span cannot exceed 360 degrees"
            )

        if self.output_angle_increment_parameter < 0.0:
            raise ValueError(
                "output_angle_increment must be 0 or positive"
            )

        if self.output_range_min < 0.0:
            raise ValueError("range_min must not be negative")

        if self.output_range_max <= self.output_range_min:
            raise ValueError(
                "range_max must be larger than range_min"
            )

        if self.sync_slop < 0.0:
            raise ValueError("sync_slop must not be negative")

        if self.sync_queue_size < 1:
            raise ValueError(
                "sync_queue_size must be at least 1"
            )

        if self.tf_timeout < 0.0:
            raise ValueError("tf_timeout must not be negative")

    def merge_callback(
        self,
        scan0: LaserScan,
        scan1: LaserScan,
    ) -> None:
        """Transform, bin, and merge a synchronized scan pair."""
        if not scan0.header.frame_id:
            self.get_logger().warning(
                f"{self.scan0_topic} has an empty frame_id"
            )
            return

        if not scan1.header.frame_id:
            self.get_logger().warning(
                f"{self.scan1_topic} has an empty frame_id"
            )
            return

        try:
            transform0 = self.tf_buffer.lookup_transform(
                self.target_frame,
                scan0.header.frame_id,
                Time.from_msg(scan0.header.stamp),
                timeout=Duration(seconds=self.tf_timeout),
            )

            transform1 = self.tf_buffer.lookup_transform(
                self.target_frame,
                scan1.header.frame_id,
                Time.from_msg(scan1.header.stamp),
                timeout=Duration(seconds=self.tf_timeout),
            )

        except TransformException as error:
            self.get_logger().warning(
                "Cannot merge scans because TF lookup failed: "
                f"{error}"
            )
            return

        angle_increment = (
            self._choose_output_angle_increment(
                scan0,
                scan1,
            )
        )

        bin_count, output_angle_max, full_circle = (
            self._create_output_geometry(angle_increment)
        )

        merged_ranges = np.full(
            bin_count,
            np.inf,
            dtype=np.float64,
        )

        self._insert_scan(
            scan=scan0,
            transform=transform0,
            merged_ranges=merged_ranges,
            angle_increment=angle_increment,
            full_circle=full_circle,
        )

        self._insert_scan(
            scan=scan1,
            transform=transform1,
            merged_ranges=merged_ranges,
            angle_increment=angle_increment,
            full_circle=full_circle,
        )

        output = LaserScan()
        output.header.stamp = self._latest_stamp(
            scan0.header.stamp,
            scan1.header.stamp,
        )
        output.header.frame_id = self.target_frame

        output.angle_min = self.output_angle_min
        output.angle_max = output_angle_max
        output.angle_increment = angle_increment

        # The output is a virtual scan produced from two sensors.
        output.time_increment = 0.0
        output.scan_time = max(
            float(scan0.scan_time),
            float(scan1.scan_time),
        )

        output.range_min = self.output_range_min
        output.range_max = self.output_range_max

        output.ranges = merged_ranges.astype(
            np.float32
        ).tolist()
        output.intensities = []

        self.scan_publisher.publish(output)

    def _choose_output_angle_increment(
        self,
        scan0: LaserScan,
        scan1: LaserScan,
    ) -> float:
        """Use the configured resolution or the finer input resolution."""
        if self.output_angle_increment_parameter > 0.0:
            return self.output_angle_increment_parameter

        candidates = [
            abs(float(scan0.angle_increment)),
            abs(float(scan1.angle_increment)),
        ]
        candidates = [
            value for value in candidates if value > 0.0
        ]

        if not candidates:
            raise ValueError(
                "Both input scans have invalid angle_increment"
            )

        return min(candidates)

    def _create_output_geometry(
        self,
        angle_increment: float,
    ) -> tuple[int, float, bool]:
        """Calculate the output array size and actual angle_max."""
        span = (
            self.output_angle_max_requested
            - self.output_angle_min
        )

        full_circle = math.isclose(
            span,
            2.0 * math.pi,
            rel_tol=0.0,
            abs_tol=max(angle_increment, 1.0e-6),
        )

        if full_circle:
            # Do not duplicate the -pi and +pi direction.
            bin_count = max(
                1,
                int(round(span / angle_increment)),
            )
        else:
            bin_count = max(
                1,
                int(math.floor(span / angle_increment)) + 1,
            )

        actual_angle_max = (
            self.output_angle_min
            + (bin_count - 1) * angle_increment
        )

        return bin_count, actual_angle_max, full_circle

    def _insert_scan(
        self,
        scan: LaserScan,
        transform,
        merged_ranges: np.ndarray,
        angle_increment: float,
        full_circle: bool,
    ) -> None:
        """Transform one scan and insert nearest ranges into output bins."""
        input_ranges = np.asarray(
            scan.ranges,
            dtype=np.float64,
        ).reshape(-1)

        if input_ranges.size == 0:
            return

        ray_indices = np.arange(
            input_ranges.size,
            dtype=np.float64,
        )

        ray_angles = (
            float(scan.angle_min)
            + ray_indices * float(scan.angle_increment)
        )

        valid = (
            np.isfinite(input_ranges)
            & (input_ranges >= float(scan.range_min))
            & (input_ranges <= float(scan.range_max))
        )

        if not np.any(valid):
            return

        valid_ranges = input_ranges[valid]
        valid_angles = ray_angles[valid]

        points_sensor = np.column_stack(
            (
                valid_ranges * np.cos(valid_angles),
                valid_ranges * np.sin(valid_angles),
                np.zeros(valid_ranges.size),
            )
        )

        rotation, translation = (
            self._transform_to_numpy(transform)
        )

        points_target = (
            points_sensor @ rotation.T
            + translation
        )

        virtual_ranges = np.hypot(
            points_target[:, 0],
            points_target[:, 1],
        )
        virtual_angles = np.arctan2(
            points_target[:, 1],
            points_target[:, 0],
        )

        valid_output = (
            np.isfinite(virtual_ranges)
            & (virtual_ranges >= self.output_range_min)
            & (virtual_ranges <= self.output_range_max)
        )

        if not np.any(valid_output):
            return

        virtual_ranges = virtual_ranges[valid_output]
        virtual_angles = virtual_angles[valid_output]

        if full_circle:
            span = (
                self.output_angle_max_requested
                - self.output_angle_min
            )

            normalized_angles = (
                (
                    virtual_angles
                    - self.output_angle_min
                )
                % span
            ) + self.output_angle_min

            output_indices = np.rint(
                (
                    normalized_angles
                    - self.output_angle_min
                )
                / angle_increment
            ).astype(np.int64)

            # A direction that rounds to the upper boundary is
            # equivalent to the first bin in a 360-degree scan.
            output_indices %= merged_ranges.size

        else:
            output_indices = np.rint(
                (
                    virtual_angles
                    - self.output_angle_min
                )
                / angle_increment
            ).astype(np.int64)

            inside = (
                (output_indices >= 0)
                & (output_indices < merged_ranges.size)
                & (
                    virtual_angles
                    >= self.output_angle_min
                )
                & (
                    virtual_angles
                    <= self.output_angle_max_requested
                )
            )

            output_indices = output_indices[inside]
            virtual_ranges = virtual_ranges[inside]

        if output_indices.size == 0:
            return

        # If two points occupy the same angle bin, keep the nearer one.
        np.minimum.at(
            merged_ranges,
            output_indices,
            virtual_ranges,
        )

    @staticmethod
    def _transform_to_numpy(transform):
        """Convert geometry_msgs/TransformStamped to R and t."""
        translation_message = (
            transform.transform.translation
        )
        rotation_message = transform.transform.rotation

        quaternion = np.array(
            [
                rotation_message.x,
                rotation_message.y,
                rotation_message.z,
                rotation_message.w,
            ],
            dtype=np.float64,
        )

        norm = np.linalg.norm(quaternion)

        if norm < 1.0e-12:
            raise ValueError(
                "Received a zero-length TF quaternion"
            )

        x, y, z, w = quaternion / norm

        rotation = np.array(
            [
                [
                    1.0 - 2.0 * (y * y + z * z),
                    2.0 * (x * y - z * w),
                    2.0 * (x * z + y * w),
                ],
                [
                    2.0 * (x * y + z * w),
                    1.0 - 2.0 * (x * x + z * z),
                    2.0 * (y * z - x * w),
                ],
                [
                    2.0 * (x * z - y * w),
                    2.0 * (y * z + x * w),
                    1.0 - 2.0 * (x * x + y * y),
                ],
            ],
            dtype=np.float64,
        )

        translation = np.array(
            [
                translation_message.x,
                translation_message.y,
                translation_message.z,
            ],
            dtype=np.float64,
        )

        return rotation, translation

    @staticmethod
    def _latest_stamp(stamp0, stamp1):
        stamp0_ns = (
            int(stamp0.sec) * 1_000_000_000
            + int(stamp0.nanosec)
        )
        stamp1_ns = (
            int(stamp1.sec) * 1_000_000_000
            + int(stamp1.nanosec)
        )

        return stamp0 if stamp0_ns >= stamp1_ns else stamp1


def main(args=None) -> None:
    rclpy.init(args=args)

    node = None

    try:
        node = ScanMergerNode()
        rclpy.spin(node)

    except KeyboardInterrupt:
        pass

    except Exception as error:
        if node is not None:
            node.get_logger().error(str(error))
        else:
            print(f"scan_merger_node error: {error}")
        raise

    finally:
        if node is not None:
            node.destroy_node()

        if rclpy.ok():
            rclpy.shutdown()


if __name__ == "__main__":
    main()