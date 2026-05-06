import math

import rclpy
from px4_msgs.msg import (
    OffboardControlMode,
    TrajectorySetpoint,
    VehicleCommand,
    VehicleLocalPosition,
    VehicleStatus,
)
from rclpy.executors import ExternalShutdownException
from rclpy.node import Node
from rclpy.qos import (
    DurabilityPolicy,
    HistoryPolicy,
    QoSProfile,
    ReliabilityPolicy,
)


class ExplorationOffboard(Node):
    def __init__(self):
        super().__init__("exploration_offboard")

        self.declare_parameter("heartbeat_rate_hz", 10.0)
        self.declare_parameter("target_x_m", 0.0)
        self.declare_parameter("target_y_m", 0.0)
        self.declare_parameter("takeoff_height_m", 2.0)
        self.declare_parameter("target_yaw_rad", 0.0)
        self.declare_parameter("auto_engage", False)

        self.heartbeat_rate_hz = float(
            self.get_parameter("heartbeat_rate_hz").value
        )
        self.target_x_m = float(self.get_parameter("target_x_m").value)
        self.target_y_m = float(self.get_parameter("target_y_m").value)
        self.takeoff_height_m = float(
            self.get_parameter("takeoff_height_m").value
        )
        self.target_yaw_rad = float(
            self.get_parameter("target_yaw_rad").value
        )
        self.auto_engage = bool(self.get_parameter("auto_engage").value)

        self.vehicle_status = None
        self.local_position = None
        self.offboard_setpoint_counter = 0
        self.offboard_enabled = False
        self.arm_command_sent = False
        px4_qos = QoSProfile(
            reliability=ReliabilityPolicy.BEST_EFFORT,
            durability=DurabilityPolicy.TRANSIENT_LOCAL,
            history=HistoryPolicy.KEEP_LAST,
            depth=1,
        )

        self.status_sub = self.create_subscription(
            VehicleStatus,
            "/fmu/out/vehicle_status",
            self.vehicle_status_callback,
            px4_qos,
        )
        self.local_position_sub = self.create_subscription(
            VehicleLocalPosition,
            "/fmu/out/vehicle_local_position_v1",
            self.vehicle_local_position_callback,
            px4_qos,
        )

        self.offboard_control_mode_pub = self.create_publisher(
            OffboardControlMode,
            "/fmu/in/offboard_control_mode",
            px4_qos,
        )
        self.trajectory_setpoint_pub = self.create_publisher(
            TrajectorySetpoint,
            "/fmu/in/trajectory_setpoint",
            px4_qos,
        )
        self.vehicle_command_pub = self.create_publisher(
            VehicleCommand,
            "/fmu/in/vehicle_command",
            px4_qos,
        )

        self.timer = self.create_timer(
            1.0 / self.heartbeat_rate_hz,
            self.timer_callback,
        )

        self.get_logger().info(
            "Exploration offboard scaffold started. "
            f"auto_engage={self.auto_engage}"
        )

    def vehicle_status_callback(self, msg: VehicleStatus):
        self.vehicle_status = msg

    def vehicle_local_position_callback(self, msg: VehicleLocalPosition):
        self.local_position = msg

    def timer_callback(self):
        timestamp_us = self.now_us()
        self.publish_offboard_heartbeat(timestamp_us)

        if self.has_valid_local_position():
            self.publish_hover_setpoint(timestamp_us)
        else:
            self.publish_takeoff_setpoint(timestamp_us)

        if not self.auto_engage:
            return

        if self.offboard_setpoint_counter < 10:
            self.offboard_setpoint_counter += 1
            return

        if not self.offboard_enabled:
            self.switch_to_offboard_mode()
            self.offboard_enabled = True

        if not self.arm_command_sent and self.preflight_checks_passed():
            self.arm()
            self.arm_command_sent = True

    def now_us(self) -> int:
        return int(self.get_clock().now().nanoseconds / 1000)

    def has_valid_local_position(self) -> bool:
        if self.local_position is None:
            return False
        return bool(
            self.local_position.xy_valid and self.local_position.z_valid
        )

    def preflight_checks_passed(self) -> bool:
        return bool(
            self.vehicle_status is not None
            and self.vehicle_status.pre_flight_checks_pass
        )

    def publish_offboard_heartbeat(self, timestamp_us: int):
        msg = OffboardControlMode()
        msg.timestamp = timestamp_us
        msg.position = True
        msg.velocity = False
        msg.acceleration = False
        msg.attitude = False
        msg.body_rate = False
        msg.thrust_and_torque = False
        msg.direct_actuator = False
        self.offboard_control_mode_pub.publish(msg)

    def make_position_setpoint(
        self,
        timestamp_us: int,
        x_m: float,
        y_m: float,
        z_m: float,
        yaw_rad: float,
    ) -> TrajectorySetpoint:
        msg = TrajectorySetpoint()
        msg.timestamp = timestamp_us
        msg.position = [float(x_m), float(y_m), float(z_m)]
        msg.velocity = [math.nan, math.nan, math.nan]
        msg.acceleration = [math.nan, math.nan, math.nan]
        msg.jerk = [math.nan, math.nan, math.nan]
        msg.yaw = float(yaw_rad)
        msg.yawspeed = math.nan
        return msg

    def publish_takeoff_setpoint(self, timestamp_us: int):
        setpoint = self.make_position_setpoint(
            timestamp_us=timestamp_us,
            x_m=self.target_x_m,
            y_m=self.target_y_m,
            z_m=-abs(self.takeoff_height_m),
            yaw_rad=self.target_yaw_rad,
        )
        self.trajectory_setpoint_pub.publish(setpoint)

    def publish_hover_setpoint(self, timestamp_us: int):
        current_z = float(self.local_position.z)
        desired_z = min(current_z, -abs(self.takeoff_height_m))
        setpoint = self.make_position_setpoint(
            timestamp_us=timestamp_us,
            x_m=self.target_x_m,
            y_m=self.target_y_m,
            z_m=desired_z,
            yaw_rad=self.target_yaw_rad,
        )
        self.trajectory_setpoint_pub.publish(setpoint)

    def publish_vehicle_command(
        self,
        command: int,
        param1: float = 0.0,
        param2: float = 0.0,
    ):
        msg = VehicleCommand()
        msg.timestamp = self.now_us()
        msg.command = command
        msg.param1 = float(param1)
        msg.param2 = float(param2)
        msg.target_system = 1
        msg.target_component = 1
        msg.source_system = 1
        msg.source_component = 1
        msg.from_external = True
        self.vehicle_command_pub.publish(msg)

    def arm(self):
        self.publish_vehicle_command(
            VehicleCommand.VEHICLE_CMD_COMPONENT_ARM_DISARM,
            param1=float(VehicleCommand.ARMING_ACTION_ARM),
        )
        self.get_logger().info("Sent arm command")

    def switch_to_offboard_mode(self):
        self.publish_vehicle_command(
            VehicleCommand.VEHICLE_CMD_DO_SET_MODE,
            param1=1.0,
            param2=6.0,
        )
        self.get_logger().info("Sent offboard mode command")


def main(args=None):
    rclpy.init(args=args)
    node = ExplorationOffboard()
    try:
        rclpy.spin(node)
    except (KeyboardInterrupt, ExternalShutdownException):
        pass
    finally:
        node.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()


if __name__ == "__main__":
    main()
