import math
from typing import Optional, Tuple

import rclpy
from geometry_msgs.msg import Point
from nav_msgs.msg import OccupancyGrid
from px4_msgs.msg import (
    BatteryStatus,
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
from sensor_msgs.msg import LaserScan
from std_msgs.msg import ColorRGBA
from visualization_msgs.msg import Marker, MarkerArray


class ExplorationOffboard(Node):
    def __init__(self):
        super().__init__("exploration_offboard")

        self.declare_parameter("heartbeat_rate_hz", 30.0)
        self.declare_parameter("takeoff_height_m", 1.0)
        self.declare_parameter("target_yaw_rad", 0.0)
        self.declare_parameter("auto_engage", False)
        self.declare_parameter("warmup_setpoint_count", 10)
        self.declare_parameter("takeoff_reached_tolerance_m", 0.10)
        self.declare_parameter("hover_velocity_tolerance_mps", 0.15)
        self.declare_parameter("hover_lateral_velocity_tolerance_mps", 0.10)
        self.declare_parameter("settle_duration_s", 1.0)
        self.declare_parameter("explore_after_hover", True)
        self.declare_parameter("entry_distance_m", 3.0)
        self.declare_parameter("setpoint_speed_mps", 0.25)
        self.declare_parameter("vertical_setpoint_speed_mps", 0.25)
        self.declare_parameter("waypoint_reached_tolerance_m", 0.30)
        self.declare_parameter("frontier_replan_period_s", 2.0)
        self.declare_parameter("frontier_goal_min_range_m", 0.8)
        self.declare_parameter("frontier_goal_max_range_m", 4.0)
        self.declare_parameter("frontier_stride_cells", 2)
        self.declare_parameter("free_cell_max_value", 20)
        self.declare_parameter("occupied_cell_min_value", 50)
        self.declare_parameter("battery_return_threshold", 0.25)
        self.declare_parameter("map_topic", "/map")
        self.declare_parameter("scripted_fallback_enabled", True)
        self.declare_parameter("scripted_forward_step_m", 1.5)
        self.declare_parameter("scripted_lateral_step_m", 0.8)
        self.declare_parameter("scripted_waypoint_count", 8)
        self.declare_parameter("lidar_obstacle_avoidance_enabled", True)
        self.declare_parameter("lidar_topic", "/drone/lidar_scan")
        self.declare_parameter("lidar_front_sector_deg", 35.0)
        self.declare_parameter("lidar_side_sector_deg", 95.0)
        self.declare_parameter("lidar_avoid_distance_m", 1.5)
        self.declare_parameter("lidar_hard_stop_distance_m", 0.75)
        self.declare_parameter("lidar_side_clearance_m", 0.65)
        self.declare_parameter("lidar_local_step_m", 0.55)
        self.declare_parameter("lidar_sidestep_m", 0.35)
        self.declare_parameter("marker_frame_id", "map")

        self.heartbeat_rate_hz = float(
            self.get_parameter("heartbeat_rate_hz").value
        )
        self.takeoff_height_m = abs(
            float(self.get_parameter("takeoff_height_m").value)
        )
        self.target_yaw_rad = float(
            self.get_parameter("target_yaw_rad").value
        )
        self.auto_engage = bool(self.get_parameter("auto_engage").value)
        self.warmup_setpoint_count = int(
            self.get_parameter("warmup_setpoint_count").value
        )
        self.takeoff_reached_tolerance_m = abs(
            float(self.get_parameter("takeoff_reached_tolerance_m").value)
        )
        self.hover_velocity_tolerance_mps = abs(
            float(self.get_parameter("hover_velocity_tolerance_mps").value)
        )
        self.hover_lateral_velocity_tolerance_mps = abs(
            float(
                self.get_parameter(
                    "hover_lateral_velocity_tolerance_mps"
                ).value
            )
        )
        self.settle_duration_us = int(
            float(self.get_parameter("settle_duration_s").value) * 1_000_000
        )
        self.explore_after_hover = bool(
            self.get_parameter("explore_after_hover").value
        )
        self.entry_distance_m = abs(
            float(self.get_parameter("entry_distance_m").value)
        )
        self.setpoint_speed_mps = abs(
            float(self.get_parameter("setpoint_speed_mps").value)
        )
        self.vertical_setpoint_speed_mps = abs(
            float(self.get_parameter("vertical_setpoint_speed_mps").value)
        )
        self.waypoint_reached_tolerance_m = abs(
            float(self.get_parameter("waypoint_reached_tolerance_m").value)
        )
        self.frontier_replan_period_us = int(
            float(self.get_parameter("frontier_replan_period_s").value)
            * 1_000_000
        )
        self.frontier_goal_min_range_m = abs(
            float(self.get_parameter("frontier_goal_min_range_m").value)
        )
        self.frontier_goal_max_range_m = abs(
            float(self.get_parameter("frontier_goal_max_range_m").value)
        )
        self.frontier_stride_cells = max(
            1, int(self.get_parameter("frontier_stride_cells").value)
        )
        self.free_cell_max_value = int(
            self.get_parameter("free_cell_max_value").value
        )
        self.occupied_cell_min_value = int(
            self.get_parameter("occupied_cell_min_value").value
        )
        self.battery_return_threshold = float(
            self.get_parameter("battery_return_threshold").value
        )
        self.map_topic = str(self.get_parameter("map_topic").value)
        self.scripted_fallback_enabled = bool(
            self.get_parameter("scripted_fallback_enabled").value
        )
        self.scripted_forward_step_m = abs(
            float(self.get_parameter("scripted_forward_step_m").value)
        )
        self.scripted_lateral_step_m = abs(
            float(self.get_parameter("scripted_lateral_step_m").value)
        )
        self.scripted_waypoint_count = max(
            1, int(self.get_parameter("scripted_waypoint_count").value)
        )
        self.lidar_obstacle_avoidance_enabled = bool(
            self.get_parameter("lidar_obstacle_avoidance_enabled").value
        )
        self.lidar_topic = str(self.get_parameter("lidar_topic").value)
        self.lidar_front_sector_rad = math.radians(
            abs(float(self.get_parameter("lidar_front_sector_deg").value))
        )
        self.lidar_side_sector_rad = math.radians(
            abs(float(self.get_parameter("lidar_side_sector_deg").value))
        )
        self.lidar_avoid_distance_m = abs(
            float(self.get_parameter("lidar_avoid_distance_m").value)
        )
        self.lidar_hard_stop_distance_m = abs(
            float(self.get_parameter("lidar_hard_stop_distance_m").value)
        )
        self.lidar_side_clearance_m = abs(
            float(self.get_parameter("lidar_side_clearance_m").value)
        )
        self.lidar_local_step_m = abs(
            float(self.get_parameter("lidar_local_step_m").value)
        )
        self.lidar_sidestep_m = abs(
            float(self.get_parameter("lidar_sidestep_m").value)
        )
        self.marker_frame_id = str(self.get_parameter("marker_frame_id").value)

        self.vehicle_status: Optional[VehicleStatus] = None
        self.local_position: Optional[VehicleLocalPosition] = None
        self.battery_status: Optional[BatteryStatus] = None
        self.latest_map: Optional[OccupancyGrid] = None
        self.lidar_front_min = math.inf
        self.lidar_left_min = math.inf
        self.lidar_right_min = math.inf
        self.lidar_scan_stamp_us = 0

        self.offboard_setpoint_counter = 0
        self.takeoff_reference_x: Optional[float] = None
        self.takeoff_reference_y: Optional[float] = None
        self.takeoff_reference_z: Optional[float] = None
        self.reference_yaw: Optional[float] = None
        self.hover_reference_x: Optional[float] = None
        self.hover_reference_y: Optional[float] = None
        self.hover_reference_z: Optional[float] = None
        self.hover_reference_yaw: Optional[float] = None
        self.target_z: Optional[float] = None

        self.commanded_x: Optional[float] = None
        self.commanded_y: Optional[float] = None
        self.commanded_z: Optional[float] = None

        self.entry_target_x: Optional[float] = None
        self.entry_target_y: Optional[float] = None
        self.frontier_target_x: Optional[float] = None
        self.frontier_target_y: Optional[float] = None
        self.last_frontier_plan_us = 0
        self.scripted_waypoints = []
        self.scripted_waypoint_index = 0

        self.takeoff_altitude_reached = False
        self.entry_completed = False
        self.return_started = False
        self.flight_phase = "WAIT_FOR_VALID_STATE"
        self.last_reported_phase: Optional[str] = None
        self.last_warn_time_us = 0
        self.last_mode_request_us = 0
        self.last_arm_request_us = 0
        self.settle_start_us = 0
        self.settle_xy_reset_counter: Optional[int] = None
        self.last_takeoff_debug_us = 0
        self.last_obstacle_log_us = 0

        px4_qos = QoSProfile(
            reliability=ReliabilityPolicy.BEST_EFFORT,
            durability=DurabilityPolicy.TRANSIENT_LOCAL,
            history=HistoryPolicy.KEEP_LAST,
            depth=1,
        )

        self.status_sub = self.create_subscription(
            VehicleStatus,
            "/fmu/out/vehicle_status_v4",
            self.vehicle_status_callback,
            px4_qos,
        )
        self.local_position_sub = self.create_subscription(
            VehicleLocalPosition,
            "/fmu/out/vehicle_local_position_v1",
            self.vehicle_local_position_callback,
            px4_qos,
        )
        self.battery_sub = self.create_subscription(
            BatteryStatus,
            "/fmu/out/battery_status_v1",
            self.battery_status_callback,
            px4_qos,
        )
        self.map_sub = self.create_subscription(
            OccupancyGrid,
            self.map_topic,
            self.map_callback,
            10,
        )
        self.lidar_sub = self.create_subscription(
            LaserScan,
            self.lidar_topic,
            self.lidar_callback,
            10,
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
        self.marker_pub = self.create_publisher(
            MarkerArray,
            "/exploration/markers",
            10,
        )

        self.timer = self.create_timer(
            1.0 / self.heartbeat_rate_hz,
            self.timer_callback,
        )

        self.get_logger().info(
            "Exploration offboard started with takeoff, cave-entry, "
            "frontier-target, and battery-return phases"
        )

    def vehicle_status_callback(self, msg: VehicleStatus):
        self.vehicle_status = msg

    def vehicle_local_position_callback(self, msg: VehicleLocalPosition):
        self.local_position = msg

    def battery_status_callback(self, msg: BatteryStatus):
        self.battery_status = msg

    def map_callback(self, msg: OccupancyGrid):
        self.latest_map = msg

    def lidar_callback(self, msg: LaserScan):
        front_min = math.inf
        left_min = math.inf
        right_min = math.inf

        angle = float(msg.angle_min)
        for value in msg.ranges:
            distance = float(value)
            valid = (
                math.isfinite(distance)
                and distance >= float(msg.range_min)
                and distance <= float(msg.range_max)
            )
            if valid:
                abs_angle = abs(angle)
                if abs_angle <= self.lidar_front_sector_rad:
                    front_min = min(front_min, distance)
                elif (
                    angle > self.lidar_front_sector_rad
                    and angle <= self.lidar_side_sector_rad
                ):
                    left_min = min(left_min, distance)
                elif (
                    angle < -self.lidar_front_sector_rad
                    and angle >= -self.lidar_side_sector_rad
                ):
                    right_min = min(right_min, distance)
            angle += float(msg.angle_increment)

        self.lidar_front_min = front_min
        self.lidar_left_min = left_min
        self.lidar_right_min = right_min
        stamp_us = (
            int(msg.header.stamp.sec) * 1_000_000
            + int(msg.header.stamp.nanosec) // 1000
        )
        self.lidar_scan_stamp_us = stamp_us if stamp_us > 0 else self.now_us()

    def timer_callback(self):
        timestamp_us = self.now_us()
        self.publish_offboard_heartbeat(timestamp_us)
        self.publish_visualization_markers(timestamp_us)

        if not self.has_valid_takeoff_state():
            self.set_flight_phase("WAIT_FOR_VALID_STATE")
            self.log_wait_condition(
                timestamp_us,
                "Waiting for PX4 vertical position and heading control state",
            )
            return

        if self.takeoff_reference_z is None:
            self.capture_takeoff_reference()

        current_z = float(self.local_position.z)
        current_vz = float(self.local_position.vz)

        if self.offboard_setpoint_counter < self.warmup_setpoint_count:
            self.set_flight_phase("WARMUP")
            self.publish_hold_setpoint(
                timestamp_us,
                self.takeoff_reference_x,
                self.takeoff_reference_y,
                self.takeoff_reference_z,
                self.reference_yaw,
                ramp=False,
            )
            self.offboard_setpoint_counter += 1
            return

        if not self.in_offboard_mode():
            self.set_flight_phase("REQUESTING_OFFBOARD")
            self.publish_hold_setpoint(
                timestamp_us,
                self.takeoff_reference_x,
                self.takeoff_reference_y,
                self.takeoff_reference_z,
                self.reference_yaw,
                ramp=False,
            )
            if timestamp_us - self.last_mode_request_us >= 1_000_000:
                self.switch_to_offboard_mode()
                self.last_mode_request_us = timestamp_us
            return

        if not self.is_armed():
            self.set_flight_phase("REQUESTING_ARM")
            self.publish_hold_setpoint(
                timestamp_us,
                self.takeoff_reference_x,
                self.takeoff_reference_y,
                self.takeoff_reference_z,
                self.reference_yaw,
                ramp=False,
            )
            if (
                self.auto_engage
                and self.preflight_checks_passed()
                and timestamp_us - self.last_arm_request_us >= 1_000_000
            ):
                self.arm()
                self.last_arm_request_us = timestamp_us
            if not self.preflight_checks_passed():
                self.log_wait_condition(
                    timestamp_us,
                    "Waiting for PX4 preflight checks before arming "
                    f"(xy_valid={self.local_position.xy_valid}, "
                    f"v_xy_valid={self.local_position.v_xy_valid}, "
                    f"heading_good_for_control={self.local_position.heading_good_for_control})",
                )
            return

        if not self.takeoff_altitude_reached:
            if self.at_target_altitude(current_z, current_vz):
                self.takeoff_altitude_reached = True
                self.settle_start_us = 0
                self.settle_xy_reset_counter = None
            else:
                self.set_flight_phase("VERTICAL_TAKEOFF")
                self.settle_start_us = 0
                self.settle_xy_reset_counter = None
                self.publish_hold_setpoint(
                    timestamp_us,
                    self.takeoff_reference_x,
                    self.takeoff_reference_y,
                    self.target_z,
                    self.reference_yaw,
                    ramp=True,
                )
                self.log_takeoff_debug(timestamp_us)
                return

        if self.hover_reference_x is None:
            self.set_flight_phase("POST_TAKEOFF_SETTLE")
            self.publish_hold_setpoint(
                timestamp_us,
                self.takeoff_reference_x,
                self.takeoff_reference_y,
                self.target_z,
                self.reference_yaw,
                ramp=True,
            )
            self.log_takeoff_debug(timestamp_us)
            if self.hover_state_is_stable(timestamp_us):
                self.capture_hover_reference()
            return

        if not self.explore_after_hover:
            self.set_flight_phase("HOVER")
            self.publish_hold_setpoint(
                timestamp_us,
                self.hover_reference_x,
                self.hover_reference_y,
                self.hover_reference_z,
                self.hover_reference_yaw,
                ramp=True,
            )
            return

        if self.should_return_home():
            self.return_started = True

        if self.return_started:
            self.run_return_home(timestamp_us)
            return

        if not self.entry_completed:
            self.run_cave_entry(timestamp_us)
            return

        self.run_frontier_exploration(timestamp_us)

    def now_us(self) -> int:
        return int(self.get_clock().now().nanoseconds / 1000)

    def has_valid_takeoff_state(self) -> bool:
        if self.vehicle_status is None or self.local_position is None:
            return False

        return bool(
            self.local_position.z_valid
            and self.local_position.v_z_valid
            and self.local_position.heading_good_for_control
        )

    def has_valid_hover_state(self) -> bool:
        if self.local_position is None:
            return False

        return bool(
            self.local_position.xy_valid
            and self.local_position.z_valid
            and self.local_position.v_xy_valid
            and self.local_position.v_z_valid
            and self.local_position.heading_good_for_control
        )

    def is_armed(self) -> bool:
        return bool(
            self.vehicle_status is not None
            and self.vehicle_status.arming_state
            == VehicleStatus.ARMING_STATE_ARMED
        )

    def in_offboard_mode(self) -> bool:
        return bool(
            self.vehicle_status is not None
            and self.vehicle_status.nav_state
            == VehicleStatus.NAVIGATION_STATE_OFFBOARD
        )

    def preflight_checks_passed(self) -> bool:
        return bool(
            self.vehicle_status is not None
            and self.vehicle_status.pre_flight_checks_pass
        )

    def capture_takeoff_reference(self):
        self.takeoff_reference_x = float(self.local_position.x)
        self.takeoff_reference_y = float(self.local_position.y)
        self.takeoff_reference_z = float(self.local_position.z)
        self.reference_yaw = float(self.local_position.heading)
        self.target_z = self.takeoff_reference_z - self.takeoff_height_m
        self.commanded_x = self.takeoff_reference_x
        self.commanded_y = self.takeoff_reference_y
        self.commanded_z = self.takeoff_reference_z
        self.takeoff_altitude_reached = False
        self.get_logger().info(
            "Latched takeoff reference: "
            f"x={self.takeoff_reference_x:.2f}, "
            f"y={self.takeoff_reference_y:.2f}, "
            f"z={self.takeoff_reference_z:.2f}, "
            f"yaw={self.reference_yaw:.2f}, "
            f"target_z={self.target_z:.2f}"
        )

    def capture_hover_reference(self):
        # Latch the actual PX4 position after settling. Pulling back to the
        # original takeoff x/y after estimator lag was causing growing
        # lateral oscillations.
        self.hover_reference_x = float(self.local_position.x)
        self.hover_reference_y = float(self.local_position.y)
        self.hover_reference_z = float(self.target_z)
        self.hover_reference_yaw = float(self.reference_yaw)
        self.commanded_x = self.hover_reference_x
        self.commanded_y = self.hover_reference_y
        self.commanded_z = self.hover_reference_z
        self.get_logger().info(
            "Latched hover reference after settle: "
            f"x={self.hover_reference_x:.2f}, y={self.hover_reference_y:.2f}, "
            f"z={self.hover_reference_z:.2f}, yaw={self.hover_reference_yaw:.2f}"
        )

    def run_cave_entry(self, timestamp_us: int):
        if self.entry_target_x is None or self.entry_target_y is None:
            yaw = float(self.hover_reference_yaw)
            self.entry_target_x = (
                float(self.hover_reference_x)
                + self.entry_distance_m * math.cos(yaw)
            )
            self.entry_target_y = (
                float(self.hover_reference_y)
                + self.entry_distance_m * math.sin(yaw)
            )
            self.get_logger().info(
                "Cave entry target: "
                f"x={self.entry_target_x:.2f}, y={self.entry_target_y:.2f}, "
                f"z={self.hover_reference_z:.2f}"
            )

        self.set_flight_phase("ENTER_CAVE")
        self.publish_hold_setpoint(
            timestamp_us,
            self.entry_target_x,
            self.entry_target_y,
            self.hover_reference_z,
            self.hover_reference_yaw,
            ramp=True,
        )
        if self.reached_xy_target(self.entry_target_x, self.entry_target_y):
            self.entry_completed = True
            self.frontier_target_x = None
            self.frontier_target_y = None
            self.get_logger().info("Cave entry complete; starting frontier exploration")

    def run_frontier_exploration(self, timestamp_us: int):
        if self.latest_map is None:
            if self.scripted_fallback_enabled:
                self.run_scripted_exploration(timestamp_us, "SCRIPTED_EXPLORE_NO_MAP")
            else:
                self.set_flight_phase("WAITING_FOR_FRONTIER_MAP")
                self.publish_hold_setpoint(
                    timestamp_us,
                    self.commanded_x,
                    self.commanded_y,
                    self.hover_reference_z,
                    self.hover_reference_yaw,
                    ramp=True,
                )
                self.log_wait_condition(
                    timestamp_us,
                    f"Waiting for occupancy grid on {self.map_topic}",
                )
            return

        should_replan = (
            self.frontier_target_x is None
            or self.frontier_target_y is None
            or self.reached_xy_target(
                self.frontier_target_x,
                self.frontier_target_y,
            )
            or timestamp_us - self.last_frontier_plan_us
            >= self.frontier_replan_period_us
        )

        if should_replan:
            frontier = self.find_frontier_goal()
            self.last_frontier_plan_us = timestamp_us
            if frontier is None:
                if self.scripted_fallback_enabled:
                    self.run_scripted_exploration(
                        timestamp_us,
                        "SCRIPTED_EXPLORE_NO_FRONTIER",
                    )
                else:
                    self.set_flight_phase("NO_FRONTIER_HOVER")
                    self.publish_hold_setpoint(
                        timestamp_us,
                        self.commanded_x,
                        self.commanded_y,
                        self.hover_reference_z,
                        self.hover_reference_yaw,
                        ramp=True,
                    )
                    self.log_wait_condition(timestamp_us, "No reachable frontier found")
                return

            self.frontier_target_x, self.frontier_target_y = frontier
            self.get_logger().info(
                "New frontier target: "
                f"x={self.frontier_target_x:.2f}, "
                f"y={self.frontier_target_y:.2f}"
            )

        self.set_flight_phase("FRONTIER_EXPLORE")
        self.publish_hold_setpoint(
            timestamp_us,
            self.frontier_target_x,
            self.frontier_target_y,
            self.hover_reference_z,
            self.hover_reference_yaw,
            ramp=True,
        )

    def run_scripted_exploration(self, timestamp_us: int, phase: str):
        if not self.scripted_waypoints:
            self.scripted_waypoints = self.make_scripted_waypoints()
            self.scripted_waypoint_index = 0
            self.get_logger().info(
                f"Using scripted exploration fallback with "
                f"{len(self.scripted_waypoints)} waypoints"
            )

        target_x, target_y = self.scripted_waypoints[self.scripted_waypoint_index]
        if self.reached_xy_target(target_x, target_y):
            if self.scripted_waypoint_index < len(self.scripted_waypoints) - 1:
                self.scripted_waypoint_index += 1
                target_x, target_y = self.scripted_waypoints[
                    self.scripted_waypoint_index
                ]
                self.get_logger().info(
                    "Scripted exploration waypoint -> "
                    f"{self.scripted_waypoint_index + 1}/"
                    f"{len(self.scripted_waypoints)}: "
                    f"x={target_x:.2f}, y={target_y:.2f}"
                )
            else:
                self.return_started = True
                self.get_logger().info(
                    "Scripted exploration complete; returning home"
                )
                self.run_return_home(timestamp_us)
                return

        self.set_flight_phase(phase)
        self.publish_hold_setpoint(
            timestamp_us,
            target_x,
            target_y,
            self.hover_reference_z,
            self.hover_reference_yaw,
            ramp=True,
        )

    def make_scripted_waypoints(self):
        yaw = float(self.hover_reference_yaw)
        start_x = float(self.entry_target_x)
        start_y = float(self.entry_target_y)
        forward_x = math.cos(yaw)
        forward_y = math.sin(yaw)
        left_x = -math.sin(yaw)
        left_y = math.cos(yaw)

        waypoints = []
        for index in range(self.scripted_waypoint_count):
            forward = (index + 1) * self.scripted_forward_step_m
            lateral_direction = -1.0 if index % 2 else 1.0
            lateral = lateral_direction * self.scripted_lateral_step_m
            waypoints.append((
                start_x + forward * forward_x + lateral * left_x,
                start_y + forward * forward_y + lateral * left_y,
            ))
        return waypoints

    def run_return_home(self, timestamp_us: int):
        self.set_flight_phase("RETURN_HOME")
        self.publish_hold_setpoint(
            timestamp_us,
            self.takeoff_reference_x,
            self.takeoff_reference_y,
            self.hover_reference_z,
            self.hover_reference_yaw,
            ramp=True,
        )
        if self.reached_xy_target(
            self.takeoff_reference_x,
            self.takeoff_reference_y,
        ):
            self.set_flight_phase("RETURN_HOVER")

    def should_return_home(self) -> bool:
        if self.battery_status is None:
            return False

        remaining = float(self.battery_status.remaining)
        battery_low = (
            0.0 <= remaining <= self.battery_return_threshold
        )
        warning_low = (
            self.battery_status.warning >= BatteryStatus.WARNING_LOW
        )
        return bool(battery_low or warning_low)

    def find_frontier_goal(self) -> Optional[Tuple[float, float]]:
        grid = self.latest_map
        if grid is None or self.local_position is None:
            return None

        width = int(grid.info.width)
        height = int(grid.info.height)
        resolution = float(grid.info.resolution)
        if width <= 2 or height <= 2 or resolution <= 0.0:
            return None

        # RTAB-Map publishes ENU-like map x/y. The PX4 local position consumed
        # by this node is NED, using x=north and y=east.
        current_map_x = float(self.local_position.y)
        current_map_y = float(self.local_position.x)
        yaw = float(self.hover_reference_yaw)
        forward_map_x = math.sin(yaw)
        forward_map_y = math.cos(yaw)

        best_score = -math.inf
        best_goal_map: Optional[Tuple[float, float]] = None

        for y in range(1, height - 1, self.frontier_stride_cells):
            row_offset = y * width
            for x in range(1, width - 1, self.frontier_stride_cells):
                index = row_offset + x
                if grid.data[index] != -1:
                    continue

                free_neighbor = self.find_free_neighbor(grid, x, y)
                if free_neighbor is None:
                    continue

                goal_map_x, goal_map_y = self.grid_to_map_xy(
                    grid,
                    free_neighbor[0],
                    free_neighbor[1],
                )
                dx = goal_map_x - current_map_x
                dy = goal_map_y - current_map_y
                distance = math.hypot(dx, dy)
                if (
                    distance < self.frontier_goal_min_range_m
                    or distance > self.frontier_goal_max_range_m
                ):
                    continue
                if self.near_occupied_cell(grid, free_neighbor[0], free_neighbor[1]):
                    continue

                forward_projection = dx * forward_map_x + dy * forward_map_y
                score = distance + 1.5 * forward_projection
                if score > best_score:
                    best_score = score
                    best_goal_map = (goal_map_x, goal_map_y)

        if best_goal_map is None:
            return None

        # Convert map ENU x/y back to PX4 local NED x/y.
        goal_ned_x = best_goal_map[1]
        goal_ned_y = best_goal_map[0]
        return goal_ned_x, goal_ned_y

    def find_free_neighbor(
        self,
        grid: OccupancyGrid,
        cell_x: int,
        cell_y: int,
    ) -> Optional[Tuple[int, int]]:
        width = int(grid.info.width)
        for dy in (-1, 0, 1):
            for dx in (-1, 0, 1):
                if dx == 0 and dy == 0:
                    continue
                nx = cell_x + dx
                ny = cell_y + dy
                value = int(grid.data[ny * width + nx])
                if 0 <= value <= self.free_cell_max_value:
                    return nx, ny
        return None

    def near_occupied_cell(
        self,
        grid: OccupancyGrid,
        cell_x: int,
        cell_y: int,
    ) -> bool:
        width = int(grid.info.width)
        height = int(grid.info.height)
        for dy in (-1, 0, 1):
            for dx in (-1, 0, 1):
                nx = cell_x + dx
                ny = cell_y + dy
                if nx < 0 or ny < 0 or nx >= width or ny >= height:
                    return True
                value = int(grid.data[ny * width + nx])
                if value >= self.occupied_cell_min_value:
                    return True
        return False

    def grid_to_map_xy(
        self,
        grid: OccupancyGrid,
        cell_x: int,
        cell_y: int,
    ) -> Tuple[float, float]:
        origin = grid.info.origin.position
        resolution = float(grid.info.resolution)
        return (
            float(origin.x) + (float(cell_x) + 0.5) * resolution,
            float(origin.y) + (float(cell_y) + 0.5) * resolution,
        )

    def at_target_altitude(
        self,
        current_z: float,
        current_vz: float,
    ) -> bool:
        if self.target_z is None:
            return False

        return bool(
            current_z <= self.target_z + self.takeoff_reached_tolerance_m
            and abs(current_vz) <= self.hover_velocity_tolerance_mps
        )

    def reached_xy_target(
        self,
        target_x: Optional[float],
        target_y: Optional[float],
    ) -> bool:
        if (
            self.local_position is None
            or target_x is None
            or target_y is None
            or not self.local_position.xy_valid
        ):
            return False

        dx = float(self.local_position.x) - float(target_x)
        dy = float(self.local_position.y) - float(target_y)
        return math.hypot(dx, dy) <= self.waypoint_reached_tolerance_m

    def hover_state_is_stable(self, timestamp_us: int) -> bool:
        if not self.has_valid_hover_state():
            self.settle_start_us = 0
            self.settle_xy_reset_counter = None
            return False

        lateral_motion_is_small = (
            abs(float(self.local_position.vx))
            <= self.hover_lateral_velocity_tolerance_mps
            and abs(float(self.local_position.vy))
            <= self.hover_lateral_velocity_tolerance_mps
        )
        vertical_motion_is_small = (
            abs(float(self.local_position.vz))
            <= self.hover_velocity_tolerance_mps
        )

        if not (lateral_motion_is_small and vertical_motion_is_small):
            self.settle_start_us = 0
            self.settle_xy_reset_counter = None
            return False

        current_reset_counter = int(self.local_position.xy_reset_counter)

        if self.settle_xy_reset_counter != current_reset_counter:
            self.settle_xy_reset_counter = current_reset_counter
            self.settle_start_us = timestamp_us
            return False

        if self.settle_start_us == 0:
            self.settle_start_us = timestamp_us
            return False

        return (timestamp_us - self.settle_start_us) >= self.settle_duration_us

    def log_wait_condition(self, timestamp_us: int, message: str):
        if timestamp_us - self.last_warn_time_us < 5_000_000:
            return
        self.get_logger().warn(message)
        self.last_warn_time_us = timestamp_us

    def log_takeoff_debug(self, timestamp_us: int):
        if timestamp_us - self.last_takeoff_debug_us < 500_000:
            return
        if self.local_position is None:
            return

        x_error = (
            float(self.local_position.x) - float(self.takeoff_reference_x)
            if self.takeoff_reference_x is not None
            else math.nan
        )
        y_error = (
            float(self.local_position.y) - float(self.takeoff_reference_y)
            if self.takeoff_reference_y is not None
            else math.nan
        )
        z_error = (
            float(self.local_position.z) - float(self.target_z)
            if self.target_z is not None
            else math.nan
        )
        yaw_error = (
            float(self.local_position.heading) - float(self.reference_yaw)
            if self.reference_yaw is not None
            else math.nan
        )

        self.get_logger().info(
            "Takeoff debug | "
            f"phase={self.flight_phase} | "
            f"ref_x={self.takeoff_reference_x:.2f} "
            f"ref_y={self.takeoff_reference_y:.2f} "
            f"ref_z={self.target_z:.2f} "
            f"ref_yaw={self.reference_yaw:.2f} | "
            f"x={self.local_position.x:.2f} "
            f"y={self.local_position.y:.2f} "
            f"z={self.local_position.z:.2f} "
            f"yaw={self.local_position.heading:.2f} | "
            f"dx={x_error:.2f} dy={y_error:.2f} dz={z_error:.2f} dyaw={yaw_error:.2f} | "
            f"vx={self.local_position.vx:.2f} "
            f"vy={self.local_position.vy:.2f} "
            f"vz={self.local_position.vz:.2f} | "
            f"xy_reset_counter={self.local_position.xy_reset_counter} "
            f"heading_reset_counter={self.local_position.heading_reset_counter}"
        )
        self.last_takeoff_debug_us = timestamp_us

    def set_flight_phase(self, new_phase: str):
        self.flight_phase = new_phase
        if self.last_reported_phase == new_phase:
            return

        battery = "unknown"
        if self.battery_status is not None:
            battery = f"{self.battery_status.remaining:.2f}"

        if self.vehicle_status is None or self.local_position is None:
            self.get_logger().info(
                f"Flight phase -> {new_phase} | battery={battery}"
            )
        else:
            self.get_logger().info(
                "Flight phase -> "
                f"{new_phase} | "
                f"nav={self.vehicle_status.nav_state} "
                f"arm={self.vehicle_status.arming_state} "
                f"preflight={self.vehicle_status.pre_flight_checks_pass} | "
                f"x={self.local_position.x:.2f} "
                f"y={self.local_position.y:.2f} "
                f"z={self.local_position.z:.2f} "
                f"vx={self.local_position.vx:.2f} "
                f"vy={self.local_position.vy:.2f} "
                f"vz={self.local_position.vz:.2f} "
                f"yaw={self.local_position.heading:.2f} | "
                f"xy_valid={self.local_position.xy_valid} "
                f"v_xy_valid={self.local_position.v_xy_valid} "
                f"z_valid={self.local_position.z_valid} "
                f"v_z_valid={self.local_position.v_z_valid} "
                f"heading_good={self.local_position.heading_good_for_control} "
                f"battery={battery}"
            )

        self.last_reported_phase = new_phase

    def publish_visualization_markers(self, timestamp_us: int):
        markers = MarkerArray()
        stamp = self.get_clock().now().to_msg()

        delete_all = Marker()
        delete_all.header.frame_id = self.marker_frame_id
        delete_all.header.stamp = stamp
        delete_all.action = Marker.DELETEALL
        markers.markers.append(delete_all)

        marker_id = 1
        if (
            self.takeoff_reference_x is not None
            and self.takeoff_reference_y is not None
        ):
            markers.markers.append(
                self.make_sphere_marker(
                    marker_id,
                    "home",
                    self.takeoff_reference_x,
                    self.takeoff_reference_y,
                    0.15,
                    self.color(0.1, 0.8, 1.0, 1.0),
                    stamp,
                )
            )
            marker_id += 1

        if self.commanded_x is not None and self.commanded_y is not None:
            markers.markers.append(
                self.make_sphere_marker(
                    marker_id,
                    "commanded_setpoint",
                    self.commanded_x,
                    self.commanded_y,
                    0.20,
                    self.color(1.0, 0.9, 0.1, 1.0),
                    stamp,
                )
            )
            marker_id += 1

        active_target = self.current_active_target()
        if active_target is not None:
            target_x, target_y = active_target
            if target_x is not None and target_y is not None:
                markers.markers.append(
                    self.make_sphere_marker(
                        marker_id,
                        "active_target",
                        target_x,
                        target_y,
                        0.28,
                        self.color(0.1, 1.0, 0.2, 1.0),
                        stamp,
                    )
                )
                marker_id += 1

                if self.local_position is not None:
                    markers.markers.append(
                        self.make_line_marker(
                            marker_id,
                            "target_line",
                            [
                                (
                                    float(self.local_position.x),
                                    float(self.local_position.y),
                                ),
                                (target_x, target_y),
                            ],
                            0.04,
                            self.color(0.1, 1.0, 0.2, 0.9),
                            stamp,
                        )
                    )
                    marker_id += 1

        if self.scripted_waypoints:
            markers.markers.append(
                self.make_spheres_marker(
                    marker_id,
                    "scripted_waypoints",
                    self.scripted_waypoints,
                    0.16,
                    self.color(0.2, 0.4, 1.0, 0.9),
                    stamp,
                )
            )
            marker_id += 1

            markers.markers.append(
                self.make_line_marker(
                    marker_id,
                    "scripted_path",
                    self.scripted_waypoints,
                    0.025,
                    self.color(0.2, 0.4, 1.0, 0.7),
                    stamp,
                )
            )
            marker_id += 1

        if self.lidar_obstacle_avoidance_enabled and self.local_position is not None:
            markers.markers.append(
                self.make_lidar_status_marker(marker_id, stamp)
            )

        self.marker_pub.publish(markers)

    def current_active_target(self) -> Optional[Tuple[float, float]]:
        if self.return_started:
            return self.takeoff_reference_x, self.takeoff_reference_y
        if not self.entry_completed:
            return self.entry_target_x, self.entry_target_y
        if self.frontier_target_x is not None and self.frontier_target_y is not None:
            return self.frontier_target_x, self.frontier_target_y
        if self.scripted_waypoints:
            return self.scripted_waypoints[self.scripted_waypoint_index]
        return None

    def make_sphere_marker(
        self,
        marker_id: int,
        namespace: str,
        x: float,
        y: float,
        scale: float,
        color: ColorRGBA,
        stamp,
    ) -> Marker:
        marker = self.base_marker(marker_id, namespace, stamp)
        rviz_x, rviz_y = self.marker_xy_from_px4_xy(x, y)
        marker.type = Marker.SPHERE
        marker.pose.position.x = rviz_x
        marker.pose.position.y = rviz_y
        marker.pose.position.z = 0.15
        marker.pose.orientation.w = 1.0
        marker.scale.x = scale
        marker.scale.y = scale
        marker.scale.z = scale
        marker.color = color
        return marker

    def make_spheres_marker(
        self,
        marker_id: int,
        namespace: str,
        points_xy,
        scale: float,
        color: ColorRGBA,
        stamp,
    ) -> Marker:
        marker = self.base_marker(marker_id, namespace, stamp)
        marker.type = Marker.SPHERE_LIST
        marker.scale.x = scale
        marker.scale.y = scale
        marker.scale.z = scale
        marker.color = color
        marker.points = [
            self.make_marker_point_from_px4_xy(x, y, 0.12)
            for x, y in points_xy
        ]
        return marker

    def make_line_marker(
        self,
        marker_id: int,
        namespace: str,
        points_xy,
        scale: float,
        color: ColorRGBA,
        stamp,
    ) -> Marker:
        marker = self.base_marker(marker_id, namespace, stamp)
        marker.type = Marker.LINE_STRIP
        marker.scale.x = scale
        marker.color = color
        marker.points = [
            self.make_marker_point_from_px4_xy(x, y, 0.12)
            for x, y in points_xy
        ]
        return marker

    def make_lidar_status_marker(self, marker_id: int, stamp) -> Marker:
        marker = self.base_marker(marker_id, "lidar_status", stamp)
        marker.type = Marker.TEXT_VIEW_FACING
        rviz_x, rviz_y = self.marker_xy_from_px4_xy(
            self.local_position.x,
            self.local_position.y,
        )
        marker.pose.position.x = rviz_x
        marker.pose.position.y = rviz_y
        marker.pose.position.z = 0.7
        marker.pose.orientation.w = 1.0
        marker.scale.z = 0.25
        obstacle = (
            self.lidar_front_min < self.lidar_avoid_distance_m
            or self.lidar_left_min < self.lidar_side_clearance_m
            or self.lidar_right_min < self.lidar_side_clearance_m
        )
        marker.color = (
            self.color(1.0, 0.1, 0.1, 1.0)
            if obstacle
            else self.color(1.0, 1.0, 1.0, 0.8)
        )
        marker.text = (
            f"{self.flight_phase}\n"
            f"front {self.format_range(self.lidar_front_min)}m | "
            f"L {self.format_range(self.lidar_left_min)}m | "
            f"R {self.format_range(self.lidar_right_min)}m"
        )
        return marker

    def base_marker(self, marker_id: int, namespace: str, stamp) -> Marker:
        marker = Marker()
        marker.header.frame_id = self.marker_frame_id
        marker.header.stamp = stamp
        marker.ns = namespace
        marker.id = marker_id
        marker.action = Marker.ADD
        return marker

    def make_marker_point_from_px4_xy(
        self,
        x: float,
        y: float,
        z: float,
    ) -> Point:
        rviz_x, rviz_y = self.marker_xy_from_px4_xy(x, y)
        return self.make_point(rviz_x, rviz_y, z)

    def marker_xy_from_px4_xy(self, x: float, y: float) -> Tuple[float, float]:
        # PX4 local_position is NED: x=north, y=east. RTAB-Map/RViz map and
        # odom frames are ENU-like: x=east, y=north.
        if self.marker_frame_id in ("map", "odom", "visual_odom"):
            return float(y), float(x)
        return float(x), float(y)

    @staticmethod
    def make_point(x: float, y: float, z: float) -> Point:
        point = Point()
        point.x = float(x)
        point.y = float(y)
        point.z = float(z)
        return point

    @staticmethod
    def color(r: float, g: float, b: float, a: float) -> ColorRGBA:
        color = ColorRGBA()
        color.r = float(r)
        color.g = float(g)
        color.b = float(b)
        color.a = float(a)
        return color

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

    def publish_hold_setpoint(
        self,
        timestamp_us: int,
        x_m: Optional[float],
        y_m: Optional[float],
        z_m: Optional[float],
        yaw_rad: Optional[float],
        ramp: bool,
    ):
        if ramp:
            x_m, y_m = self.apply_lidar_obstacle_avoidance(
                timestamp_us,
                x_m,
                y_m,
            )

        if ramp:
            x_m = self.ramped_axis(
                self.commanded_x,
                x_m,
                self.setpoint_speed_mps,
            )
            y_m = self.ramped_axis(
                self.commanded_y,
                y_m,
                self.setpoint_speed_mps,
            )
            z_m = self.ramped_axis(
                self.commanded_z,
                z_m,
                self.vertical_setpoint_speed_mps,
            )

        self.commanded_x = x_m
        self.commanded_y = y_m
        self.commanded_z = z_m

        msg = TrajectorySetpoint()
        msg.timestamp = timestamp_us
        msg.position = [
            math.nan if x_m is None else float(x_m),
            math.nan if y_m is None else float(y_m),
            math.nan if z_m is None else float(z_m),
        ]
        msg.velocity = [math.nan, math.nan, math.nan]
        msg.acceleration = [math.nan, math.nan, math.nan]
        msg.jerk = [math.nan, math.nan, math.nan]
        msg.yaw = float(self.target_yaw_rad if yaw_rad is None else yaw_rad)
        msg.yawspeed = 0.0
        self.trajectory_setpoint_pub.publish(msg)

    def apply_lidar_obstacle_avoidance(
        self,
        timestamp_us: int,
        target_x: Optional[float],
        target_y: Optional[float],
    ) -> Tuple[Optional[float], Optional[float]]:
        if (
            not self.lidar_obstacle_avoidance_enabled
            or self.local_position is None
            or target_x is None
            or target_y is None
            or self.hover_reference_x is None
            or self.lidar_scan_stamp_us == 0
        ):
            return target_x, target_y

        # Ignore stale scans so old obstacle detections don't trap the vehicle.
        if timestamp_us - self.lidar_scan_stamp_us > 500_000:
            return target_x, target_y

        front = self.lidar_front_min
        left = self.lidar_left_min
        right = self.lidar_right_min
        obstacle_close = (
            front < self.lidar_avoid_distance_m
            or left < self.lidar_side_clearance_m
            or right < self.lidar_side_clearance_m
        )
        if not obstacle_close:
            return target_x, target_y

        current_x = float(self.local_position.x)
        current_y = float(self.local_position.y)
        yaw = float(self.local_position.heading)
        forward_x = math.cos(yaw)
        forward_y = math.sin(yaw)
        left_x = -math.sin(yaw)
        left_y = math.cos(yaw)

        dx = float(target_x) - current_x
        dy = float(target_y) - current_y
        desired_forward = dx * forward_x + dy * forward_y
        desired_left = dx * left_x + dy * left_y

        desired_forward = self.clamp(
            desired_forward,
            -self.lidar_local_step_m,
            self.lidar_local_step_m,
        )
        desired_left = self.clamp(
            desired_left,
            -self.lidar_local_step_m,
            self.lidar_local_step_m,
        )

        avoidance_reason = "obstacle"
        if front < self.lidar_hard_stop_distance_m:
            # Hard stop: don't continue forward. Pick the clearer side.
            desired_forward = min(desired_forward, 0.0)
            side_sign = 1.0 if left >= right else -1.0
            desired_left = side_sign * self.lidar_sidestep_m
            avoidance_reason = "front hard stop"
        elif front < self.lidar_avoid_distance_m:
            scale = max(
                0.0,
                (front - self.lidar_hard_stop_distance_m)
                / max(
                    self.lidar_avoid_distance_m
                    - self.lidar_hard_stop_distance_m,
                    0.001,
                ),
            )
            desired_forward = min(desired_forward, desired_forward * scale)
            side_sign = 1.0 if left >= right else -1.0
            desired_left += side_sign * self.lidar_sidestep_m * (1.0 - scale)
            avoidance_reason = "front avoidance"

        if left < self.lidar_side_clearance_m:
            desired_left -= self.lidar_sidestep_m
            avoidance_reason = "left clearance"
        if right < self.lidar_side_clearance_m:
            desired_left += self.lidar_sidestep_m
            avoidance_reason = "right clearance"

        desired_left = self.clamp(
            desired_left,
            -self.lidar_local_step_m,
            self.lidar_local_step_m,
        )

        adjusted_x = (
            current_x
            + desired_forward * forward_x
            + desired_left * left_x
        )
        adjusted_y = (
            current_y
            + desired_forward * forward_y
            + desired_left * left_y
        )

        self.log_lidar_obstacle(timestamp_us, avoidance_reason)
        return adjusted_x, adjusted_y

    def log_lidar_obstacle(self, timestamp_us: int, reason: str):
        if timestamp_us - self.last_obstacle_log_us < 1_000_000:
            return

        self.get_logger().warn(
            "Lidar obstacle avoidance active | "
            f"reason={reason} | "
            f"front={self.format_range(self.lidar_front_min)}m "
            f"left={self.format_range(self.lidar_left_min)}m "
            f"right={self.format_range(self.lidar_right_min)}m"
        )
        self.last_obstacle_log_us = timestamp_us

    @staticmethod
    def format_range(value: float) -> str:
        if not math.isfinite(value):
            return "inf"
        return f"{value:.2f}"

    @staticmethod
    def clamp(value: float, minimum: float, maximum: float) -> float:
        return max(minimum, min(maximum, value))

    def ramped_axis(
        self,
        current_command: Optional[float],
        target: Optional[float],
        speed_mps: float,
    ) -> Optional[float]:
        if target is None:
            return None
        if current_command is None:
            return float(target)

        max_step = max(speed_mps / self.heartbeat_rate_hz, 0.001)
        delta = float(target) - float(current_command)
        if abs(delta) <= max_step:
            return float(target)
        return float(current_command) + math.copysign(max_step, delta)

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
