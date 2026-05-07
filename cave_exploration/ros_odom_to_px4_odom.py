import math
import threading
from typing import Optional, Tuple

import numpy as np
import rclpy
from nav_msgs.msg import Odometry
from px4_msgs.msg import VehicleOdometry
from rclpy.node import Node


def quat_xyzw_to_rotmat(x: float, y: float, z: float, w: float) -> np.ndarray:
    q = np.array([x, y, z, w], dtype=float)
    n = np.linalg.norm(q)
    if n < 1e-12:
        return np.eye(3)
    x, y, z, w = q / n

    return np.array([
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
    ])


def rotmat_to_quat_xyzw(R: np.ndarray) -> Tuple[float, float, float, float]:
    trace = np.trace(R)
    if trace > 0.0:
        s = math.sqrt(trace + 1.0) * 2.0
        w = 0.25 * s
        x = (R[2, 1] - R[1, 2]) / s
        y = (R[0, 2] - R[2, 0]) / s
        z = (R[1, 0] - R[0, 1]) / s
    elif R[0, 0] > R[1, 1] and R[0, 0] > R[2, 2]:
        s = math.sqrt(1.0 + R[0, 0] - R[1, 1] - R[2, 2]) * 2.0
        w = (R[2, 1] - R[1, 2]) / s
        x = 0.25 * s
        y = (R[0, 1] + R[1, 0]) / s
        z = (R[0, 2] + R[2, 0]) / s
    elif R[1, 1] > R[2, 2]:
        s = math.sqrt(1.0 + R[1, 1] - R[0, 0] - R[2, 2]) * 2.0
        w = (R[0, 2] - R[2, 0]) / s
        x = (R[0, 1] + R[1, 0]) / s
        y = 0.25 * s
        z = (R[1, 2] + R[2, 1]) / s
    else:
        s = math.sqrt(1.0 + R[2, 2] - R[0, 0] - R[1, 1]) * 2.0
        w = (R[1, 0] - R[0, 1]) / s
        x = (R[0, 2] + R[2, 0]) / s
        y = (R[1, 2] + R[2, 1]) / s
        z = 0.25 * s

    q = np.array([x, y, z, w], dtype=float)
    q /= np.linalg.norm(q)
    return float(q[0]), float(q[1]), float(q[2]), float(q[3])


def enu_flu_to_ned_frd(
    pos_enu: np.ndarray, quat_xyzw_enu_flu: np.ndarray
) -> Tuple[np.ndarray, np.ndarray]:
    pos_ned = np.array([pos_enu[1], pos_enu[0], -pos_enu[2]], dtype=float)

    R_ned_enu = np.array([
        [0.0, 1.0, 0.0],
        [1.0, 0.0, 0.0],
        [0.0, 0.0, -1.0],
    ])

    R_flu_frd = np.array([
        [1.0, 0.0, 0.0],
        [0.0, -1.0, 0.0],
        [0.0, 0.0, -1.0],
    ])

    R_enu_flu = quat_xyzw_to_rotmat(
        quat_xyzw_enu_flu[0],
        quat_xyzw_enu_flu[1],
        quat_xyzw_enu_flu[2],
        quat_xyzw_enu_flu[3],
    )

    R_ned_frd = R_ned_enu @ R_enu_flu @ R_flu_frd
    quat_ned_frd = np.array(rotmat_to_quat_xyzw(R_ned_frd), dtype=float)
    return pos_ned, quat_ned_frd


class RosOdomToPx4(Node):
    def __init__(self):
        super().__init__("ros_odom_to_px4")

        self.declare_parameter("odom_topic", "/rtabmap/odom")
        self.declare_parameter("publish_rate_hz", 30.0)
        self.declare_parameter("quality", 80)
        self.declare_parameter("position_variance", [0.05, 0.05, 0.08])
        self.declare_parameter("orientation_variance", [0.03, 0.03, 0.05])
        self.declare_parameter("velocity_variance", [0.10, 0.10, 0.15])

        self.odom_topic = self.get_parameter("odom_topic").value
        publish_rate = self.get_parameter("publish_rate_hz").value
        self.publish_rate_hz = float(publish_rate)
        self.quality = int(self.get_parameter("quality").value)
        self.position_variance = list(
            self.get_parameter("position_variance").value
        )
        self.orientation_variance = list(
            self.get_parameter("orientation_variance").value
        )
        self.velocity_variance = list(
            self.get_parameter("velocity_variance").value
        )

        self.pub = self.create_publisher(
            VehicleOdometry,
            "/fmu/in/vehicle_visual_odometry",
            10,
        )
        self.sub = self.create_subscription(
            Odometry,
            self.odom_topic,
            self.odom_callback,
            10,
        )

        self.lock = threading.Lock()
        self.latest_position_ned: Optional[np.ndarray] = None
        self.latest_quat_ned_frd: Optional[np.ndarray] = None
        self.latest_stamp_us: Optional[int] = None
        self.last_position_ned: Optional[np.ndarray] = None
        self.last_stamp_us: Optional[int] = None

        self.timer = self.create_timer(
            1.0 / self.publish_rate_hz,
            self.publish_odom,
        )

        self.get_logger().info(f"Listening to: {self.odom_topic}")
        self.get_logger().info(
            "Publishing to: /fmu/in/vehicle_visual_odometry"
        )

    def odom_callback(self, msg: Odometry):
        pose = msg.pose.pose
        pos_enu = np.array(
            [pose.position.x, pose.position.y, pose.position.z],
            dtype=float,
        )
        quat_enu_flu = np.array(
            [
                pose.orientation.x,
                pose.orientation.y,
                pose.orientation.z,
                pose.orientation.w,
            ],
            dtype=float,
        )
        pos_ned, quat_ned_frd = enu_flu_to_ned_frd(pos_enu, quat_enu_flu)

        stamp_us = int(
            (msg.header.stamp.sec * 1_000_000)
            + (msg.header.stamp.nanosec // 1000)
        )
        if stamp_us == 0:
            stamp_us = int(self.get_clock().now().nanoseconds // 1000)

        with self.lock:
            self.latest_position_ned = pos_ned
            self.latest_quat_ned_frd = quat_ned_frd
            self.latest_stamp_us = stamp_us

    def publish_odom(self):
        with self.lock:
            if (
                self.latest_position_ned is None
                or self.latest_quat_ned_frd is None
            ):
                return

            position = self.latest_position_ned.copy()
            quat = self.latest_quat_ned_frd.copy()
            stamp_us = int(self.latest_stamp_us)

            if (
                self.last_position_ned is not None
                and self.last_stamp_us is not None
            ):
                dt = max((stamp_us - self.last_stamp_us) * 1e-6, 1e-6)
                vel_ned = (position - self.last_position_ned) / dt
            else:
                vel_ned = np.zeros(3, dtype=float)

            self.last_position_ned = position.copy()
            self.last_stamp_us = stamp_us

        msg = VehicleOdometry()
        msg.timestamp = stamp_us
        msg.timestamp_sample = stamp_us
        msg.pose_frame = VehicleOdometry.POSE_FRAME_NED
        msg.velocity_frame = VehicleOdometry.VELOCITY_FRAME_NED
        msg.position = [
            float(position[0]),
            float(position[1]),
            float(position[2]),
        ]
        msg.q = [
            float(quat[3]),
            float(quat[0]),
            float(quat[1]),
            float(quat[2]),
        ]
        msg.velocity = [
            float(vel_ned[0]),
            float(vel_ned[1]),
            float(vel_ned[2]),
        ]
        msg.angular_velocity = [math.nan, math.nan, math.nan]
        msg.position_variance = [float(v) for v in self.position_variance]
        msg.orientation_variance = [
            float(v) for v in self.orientation_variance
        ]
        msg.velocity_variance = [float(v) for v in self.velocity_variance]
        msg.reset_counter = 0
        msg.quality = self.quality

        self.pub.publish(msg)


def main(args=None):
    rclpy.init(args=args)
    node = RosOdomToPx4()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == "__main__":
    main()
