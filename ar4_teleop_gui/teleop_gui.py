#!/usr/bin/env python3
"""AR4 teleop GUI with joint and Cartesian jog modes."""

import math
import threading
import time
import tkinter as tk

import rclpy
from geometry_msgs.msg import TwistStamped
from moveit_msgs.srv import ServoCommandType
from rclpy.executors import MultiThreadedExecutor
from rclpy.node import Node
from sensor_msgs.msg import JointState
from trajectory_msgs.msg import JointTrajectory, JointTrajectoryPoint


JOINT_BASE_NAMES = [
    "joint_1",
    "joint_2",
    "joint_3",
    "joint_4",
    "joint_5",
    "joint_6",
]
DEFAULT_STEP_DEG = 2.0
DEFAULT_COMMAND_PERIOD = 0.15
DEFAULT_TRAJECTORY_TIME = 0.4
DEFAULT_HOLD_REPEAT_MS = 80
DEFAULT_HOLD_INITIAL_DELAY_MS = 220
DEFAULT_LINEAR_SPEED = 0.03
DEFAULT_ANGULAR_SPEED = 0.25
DEFAULT_WAYPOINT_TOLERANCE_DEG = 2.0
DEFAULT_WAYPOINT_TIMEOUT_SEC = 15.0
DEFAULT_WAYPOINT_LOG_PERIOD_SEC = 0.5
DEFAULT_WAYPOINT_REISSUE_PERIOD_SEC = 0.8
DEFAULT_WAYPOINT_SEGMENT_TIME = 1.8
DEFAULT_WAYPOINT_STALL_TIMEOUT_SEC = 2.5
DEFAULT_WAYPOINT_MAX_SPEED_DEG_S = 20.0
DEFAULT_WAYPOINT_MAX_ACCEL_DEG_S2 = 40.0
SERVO_READY_POSITION = [0.0, 0.0, -1.5708, 0.0, 0.0, 0.0]


class TeleopGUI(Node):
    def __init__(self):
        super().__init__("teleop_gui_node")

        self.declare_parameter("tf_prefix", "")
        self.declare_parameter("command_topic", "/joint_trajectory_controller/joint_trajectory")
        self.declare_parameter("cartesian_topic", "/servo_node/delta_twist_cmds")
        self.declare_parameter("servo_command_type_service", "/servo_node/switch_command_type")
        self.declare_parameter("twist_frame", "base_link")
        self.declare_parameter("command_period", DEFAULT_COMMAND_PERIOD)
        self.declare_parameter("trajectory_time", DEFAULT_TRAJECTORY_TIME)
        self.declare_parameter("linear_speed", DEFAULT_LINEAR_SPEED)
        self.declare_parameter("angular_speed", DEFAULT_ANGULAR_SPEED)
        self.declare_parameter("waypoint_tolerance_deg", DEFAULT_WAYPOINT_TOLERANCE_DEG)
        self.declare_parameter("waypoint_timeout_sec", DEFAULT_WAYPOINT_TIMEOUT_SEC)
        self.declare_parameter("waypoint_log_period_sec", DEFAULT_WAYPOINT_LOG_PERIOD_SEC)
        self.declare_parameter("waypoint_reissue_period_sec", DEFAULT_WAYPOINT_REISSUE_PERIOD_SEC)
        self.declare_parameter("waypoint_segment_time", DEFAULT_WAYPOINT_SEGMENT_TIME)
        self.declare_parameter("waypoint_stall_timeout_sec", DEFAULT_WAYPOINT_STALL_TIMEOUT_SEC)
        self.declare_parameter("waypoint_max_speed_deg_s", DEFAULT_WAYPOINT_MAX_SPEED_DEG_S)
        self.declare_parameter("waypoint_max_accel_deg_s2", DEFAULT_WAYPOINT_MAX_ACCEL_DEG_S2)

        self.tf_prefix = self.get_parameter("tf_prefix").value
        self.command_topic = self.get_parameter("command_topic").value
        self.cartesian_topic = self.get_parameter("cartesian_topic").value
        self.servo_command_type_service = self.get_parameter("servo_command_type_service").value
        self.twist_frame = self.get_parameter("twist_frame").value
        self.command_period = float(self.get_parameter("command_period").value)
        self.trajectory_time = float(self.get_parameter("trajectory_time").value)
        self.linear_speed = float(self.get_parameter("linear_speed").value)
        self.angular_speed = float(self.get_parameter("angular_speed").value)
        self.waypoint_tolerance_rad = math.radians(float(self.get_parameter("waypoint_tolerance_deg").value))
        self.waypoint_timeout_sec = float(self.get_parameter("waypoint_timeout_sec").value)
        self.waypoint_log_period_sec = float(self.get_parameter("waypoint_log_period_sec").value)
        self.waypoint_reissue_period_sec = float(self.get_parameter("waypoint_reissue_period_sec").value)
        self.waypoint_segment_time = float(self.get_parameter("waypoint_segment_time").value)
        self.waypoint_stall_timeout_sec = float(self.get_parameter("waypoint_stall_timeout_sec").value)
        self.waypoint_max_speed_deg_s = float(self.get_parameter("waypoint_max_speed_deg_s").value)
        self.waypoint_max_accel_deg_s2 = float(self.get_parameter("waypoint_max_accel_deg_s2").value)

        self.joint_names = [f"{self.tf_prefix}{name}" for name in JOINT_BASE_NAMES]
        self.current_joints = [0.0] * len(self.joint_names)
        self.target_joints = [0.0] * len(self.joint_names)
        self.waypoints = []
        self._state_received = False
        self._last_sent_target = None
        self._pending_send = False
        self._suppress_slider_callbacks = False
        self._path_running = False

        self.publisher = self.create_publisher(JointTrajectory, self.command_topic, 10)
        self.cartesian_publisher = self.create_publisher(TwistStamped, self.cartesian_topic, 10)
        self.command_type_client = self.create_client(ServoCommandType, self.servo_command_type_service)
        self.create_subscription(JointState, "/joint_states", self._joint_state_cb, 10)

        self._hold_job = None
        self._hold_joint_index = None
        self._hold_delta_deg = 0.0

        self._cart_hold_job = None
        self._cart_hold_linear = (0.0, 0.0, 0.0)
        self._cart_hold_angular = (0.0, 0.0, 0.0)
        self._servo_twist_mode_enabled = False

        self._build_gui()
        self._refresh_ui()

        self.status.config(text="Waiting for joint states...", fg="orange")
        self.root.after(int(self.command_period * 1000), self._periodic_publish)

    def _joint_state_cb(self, msg):
        index_by_name = {name: idx for idx, name in enumerate(msg.name)}
        updated = False

        for i, name in enumerate(self.joint_names):
            if name in index_by_name:
                position_index = index_by_name[name]
                self.current_joints[i] = float(msg.position[position_index])
                if not self._state_received:
                    self.target_joints[i] = self.current_joints[i]
                updated = True

        if updated and not self._state_received:
            self._state_received = True
            self._set_status("Ready", "green")
            self.root.after(0, self._sync_sliders_to_current)

    def _build_gui(self):
        self.root = tk.Tk()
        self.root.title("AR4 Joint + Cartesian Teleop")
        self.root.geometry("980x560")
        self.root.minsize(920, 520)

        header = tk.Frame(self.root)
        header.pack(fill="x", padx=12, pady=(12, 6))

        tk.Label(header, text="AR4 Teleop", font=("Arial", 16, "bold")).pack(anchor="w")
        tk.Label(
            header,
            text="Joint jog uses joint_trajectory_controller. Cartesian jog uses MoveIt Servo twist commands.",
            fg="#555",
        ).pack(anchor="w", pady=(4, 0))

        body = tk.Frame(self.root)
        body.pack(fill="both", expand=True, padx=12, pady=8)

        left = tk.Frame(body)
        left.pack(side="left", fill="both", expand=True)

        right = tk.Frame(body, width=360)
        right.pack(side="right", fill="y", padx=(12, 0))
        right.pack_propagate(False)

        self._slider_vars = []
        self._value_labels = []

        for i, base_name in enumerate(JOINT_BASE_NAMES):
            row = tk.Frame(left)
            row.pack(fill="x", pady=6)

            tk.Label(row, text=base_name, width=10, anchor="w").pack(side="left")

            minus_button = tk.Button(row, text="-", width=3, command=lambda j=i: self._jog_joint(j, -DEFAULT_STEP_DEG))
            minus_button.pack(side="left", padx=(0, 4))
            minus_button.bind("<ButtonPress-1>", lambda _event, j=i: self._start_hold_jog(j, -DEFAULT_STEP_DEG))
            minus_button.bind("<ButtonRelease-1>", self._stop_hold_jog)
            minus_button.bind("<Leave>", self._stop_hold_jog)

            slider_var = tk.DoubleVar(value=0.0)
            self._slider_vars.append(slider_var)
            slider = tk.Scale(
                row,
                variable=slider_var,
                from_=-180.0,
                to=180.0,
                resolution=0.1,
                orient="horizontal",
                length=360,
                showvalue=False,
                command=lambda _value, j=i: self._on_slider_change(j),
            )
            slider.pack(side="left", fill="x", expand=True)

            plus_button = tk.Button(row, text="+", width=3, command=lambda j=i: self._jog_joint(j, +DEFAULT_STEP_DEG))
            plus_button.pack(side="left", padx=(4, 4))
            plus_button.bind("<ButtonPress-1>", lambda _event, j=i: self._start_hold_jog(j, +DEFAULT_STEP_DEG))
            plus_button.bind("<ButtonRelease-1>", self._stop_hold_jog)
            plus_button.bind("<Leave>", self._stop_hold_jog)

            value_label = tk.Label(row, text="0.0 deg", width=8, anchor="e", fg="#005bbb")
            value_label.pack(side="left")
            self._value_labels.append(value_label)

        tk.Button(right, text="Send target", command=self._mark_for_send, bg="#d9edf7").pack(fill="x", pady=(0, 8))
        tk.Button(right, text="Sync sliders to current", command=self._sync_sliders_to_current).pack(fill="x", pady=(0, 8))
        tk.Button(right, text="Zero pose", command=self._go_zero, bg="#f0f0f0").pack(fill="x", pady=(0, 8))
        tk.Button(right, text="Hold current", command=self._hold_current).pack(fill="x", pady=(0, 8))
        tk.Button(right, text="Servo-ready pose", command=self._go_servo_ready, bg="#eef6e8").pack(fill="x", pady=(0, 8))

        cartesian_frame = tk.LabelFrame(right, text="Cartesian Jog (MoveIt Servo)")
        cartesian_frame.pack(fill="x", pady=(8, 10))

        tk.Label(
            cartesian_frame,
            text=(
                f"Frame: {self.twist_frame} | Topic: {self.cartesian_topic}\n"
                f"linear={self.linear_speed:.3f} m/s, angular={self.angular_speed:.3f} rad/s\n"
                f"mode service: {self.servo_command_type_service}"
            ),
            justify="left",
            fg="#555",
        ).grid(row=0, column=0, columnspan=4, sticky="w", padx=6, pady=(4, 6))

        self._create_cartesian_button(cartesian_frame, 1, 0, "X-", (-self.linear_speed, 0.0, 0.0), (0.0, 0.0, 0.0))
        self._create_cartesian_button(cartesian_frame, 1, 1, "X+", (self.linear_speed, 0.0, 0.0), (0.0, 0.0, 0.0))
        self._create_cartesian_button(cartesian_frame, 1, 2, "Y-", (0.0, -self.linear_speed, 0.0), (0.0, 0.0, 0.0))
        self._create_cartesian_button(cartesian_frame, 1, 3, "Y+", (0.0, self.linear_speed, 0.0), (0.0, 0.0, 0.0))
        self._create_cartesian_button(cartesian_frame, 2, 0, "Z-", (0.0, 0.0, -self.linear_speed), (0.0, 0.0, 0.0))
        self._create_cartesian_button(cartesian_frame, 2, 1, "Z+", (0.0, 0.0, self.linear_speed), (0.0, 0.0, 0.0))
        self._create_cartesian_button(cartesian_frame, 2, 2, "Mx-", (0.0, 0.0, 0.0), (-self.angular_speed, 0.0, 0.0))
        self._create_cartesian_button(cartesian_frame, 2, 3, "Mx+", (0.0, 0.0, 0.0), (self.angular_speed, 0.0, 0.0))
        self._create_cartesian_button(cartesian_frame, 3, 0, "My-", (0.0, 0.0, 0.0), (0.0, -self.angular_speed, 0.0))
        self._create_cartesian_button(cartesian_frame, 3, 1, "My+", (0.0, 0.0, 0.0), (0.0, self.angular_speed, 0.0))
        self._create_cartesian_button(cartesian_frame, 3, 2, "Mz-", (0.0, 0.0, 0.0), (0.0, 0.0, -self.angular_speed))
        self._create_cartesian_button(cartesian_frame, 3, 3, "Mz+", (0.0, 0.0, 0.0), (0.0, 0.0, self.angular_speed))

        tk.Button(cartesian_frame, text="Stop Cartesian", command=self._publish_zero_twist, bg="#ffe0e0").grid(
            row=4,
            column=0,
            columnspan=4,
            sticky="ew",
            padx=6,
            pady=(6, 6),
        )

        tk.Label(right, text="Waypoints", anchor="w", font=("Arial", 10, "bold")).pack(fill="x", pady=(6, 4))

        self.waypoint_list = tk.Listbox(right, height=7, exportselection=False)
        self.waypoint_list.pack(fill="both", expand=True)

        waypoint_buttons = tk.Frame(right)
        waypoint_buttons.pack(fill="x", pady=(8, 0))

        tk.Button(waypoint_buttons, text="Add point (current pose)", command=self._save_waypoint).pack(fill="x", pady=(0, 6))
        tk.Button(waypoint_buttons, text="Delete selected", command=self._delete_waypoint).pack(fill="x", pady=(0, 6))
        tk.Button(waypoint_buttons, text="Start path (from first)", command=self._run_waypoints, bg="#e8f5e9").pack(fill="x")

        self.status = tk.Label(right, text="Initializing...", fg="gray", justify="left", wraplength=340)
        self.status.pack(fill="x", pady=(12, 0))

    def _create_cartesian_button(self, parent, row, col, label, linear, angular):
        button = tk.Button(parent, text=label, width=7, command=lambda l=linear, a=angular: self._send_cartesian_once(l, a))
        button.grid(row=row, column=col, padx=4, pady=3, sticky="ew")
        button.bind("<ButtonPress-1>", lambda _event, l=linear, a=angular: self._start_hold_cartesian(l, a))
        button.bind("<ButtonRelease-1>", self._stop_hold_cartesian)
        button.bind("<Leave>", self._stop_hold_cartesian)

    def _refresh_ui(self):
        for i, label in enumerate(self._value_labels):
            label.config(text=f"{math.degrees(self.target_joints[i]):.1f} deg")
        self.root.after(100, self._refresh_ui)

    def _sync_sliders_to_current(self):
        self._suppress_slider_callbacks = True
        for i, slider_var in enumerate(self._slider_vars):
            slider_var.set(math.degrees(self.current_joints[i]))
            self.target_joints[i] = self.current_joints[i]
        self._suppress_slider_callbacks = False
        self._pending_send = True
        self._set_status("Synced to current state", "#444")

    def _on_slider_change(self, joint_index):
        if self._suppress_slider_callbacks or not self._state_received:
            return

        self.target_joints = list(self.current_joints)
        self.target_joints[joint_index] = math.radians(float(self._slider_vars[joint_index].get()))
        self._pending_send = True

    def _jog_joint(self, joint_index, delta_deg):
        if not self._state_received:
            self._set_status("Waiting for joint states...", "orange")
            return

        self._sync_sliders_to_current()
        current_deg = float(self._slider_vars[joint_index].get())
        self._slider_vars[joint_index].set(current_deg + delta_deg)
        self._on_slider_change(joint_index)

    def _start_hold_jog(self, joint_index, delta_deg):
        self._stop_hold_jog()
        self._hold_joint_index = joint_index
        self._hold_delta_deg = delta_deg
        self._hold_job = self.root.after(DEFAULT_HOLD_INITIAL_DELAY_MS, self._repeat_hold_jog)

    def _repeat_hold_jog(self):
        if self._hold_joint_index is None:
            return
        self._jog_joint(self._hold_joint_index, self._hold_delta_deg)
        self._hold_job = self.root.after(DEFAULT_HOLD_REPEAT_MS, self._repeat_hold_jog)

    def _stop_hold_jog(self, _event=None):
        if self._hold_job is not None:
            self.root.after_cancel(self._hold_job)
            self._hold_job = None
        self._hold_joint_index = None
        self._hold_delta_deg = 0.0

    def _send_cartesian_once(self, linear, angular):
        self._publish_twist(linear, angular)

    def _start_hold_cartesian(self, linear, angular):
        self._stop_hold_cartesian()
        self._cart_hold_linear = linear
        self._cart_hold_angular = angular
        self._cart_hold_job = self.root.after(DEFAULT_HOLD_INITIAL_DELAY_MS, self._repeat_hold_cartesian)

    def _repeat_hold_cartesian(self):
        self._publish_twist(self._cart_hold_linear, self._cart_hold_angular)
        self._cart_hold_job = self.root.after(DEFAULT_HOLD_REPEAT_MS, self._repeat_hold_cartesian)

    def _stop_hold_cartesian(self, _event=None):
        if self._cart_hold_job is not None:
            self.root.after_cancel(self._cart_hold_job)
            self._cart_hold_job = None
        self._cart_hold_linear = (0.0, 0.0, 0.0)
        self._cart_hold_angular = (0.0, 0.0, 0.0)

    def _publish_twist(self, linear, angular):
        if not self._ensure_servo_twist_mode():
            return

        message = TwistStamped()
        message.header.stamp = self.get_clock().now().to_msg()
        message.header.frame_id = self.twist_frame

        message.twist.linear.x = float(linear[0])
        message.twist.linear.y = float(linear[1])
        message.twist.linear.z = float(linear[2])
        message.twist.angular.x = float(angular[0])
        message.twist.angular.y = float(angular[1])
        message.twist.angular.z = float(angular[2])

        self.cartesian_publisher.publish(message)
        if self.cartesian_publisher.get_subscription_count() == 0:
            self._set_status("Servo not running (no subscribers on cartesian topic)", "orange")
        else:
            self._set_status("Cartesian command sent", "green")

    def _ensure_servo_twist_mode(self):
        if self._servo_twist_mode_enabled:
            return True

        if not self.command_type_client.wait_for_service(timeout_sec=0.2):
            self._set_status("Servo switch_command_type service not available", "orange")
            return False

        request = ServoCommandType.Request()
        request.command_type = ServoCommandType.Request.TWIST

        future = self.command_type_client.call_async(request)
        rclpy.spin_until_future_complete(self, future, timeout_sec=1.0)

        if not future.done() or future.result() is None:
            self._set_status("Failed to switch Servo to TWIST mode", "red")
            return False

        if not future.result().success:
            self._set_status("Servo rejected TWIST mode", "red")
            return False

        self._servo_twist_mode_enabled = True
        return True

    def _publish_zero_twist(self):
        self._stop_hold_cartesian()
        self._publish_twist((0.0, 0.0, 0.0), (0.0, 0.0, 0.0))

    def _go_zero(self):
        self._suppress_slider_callbacks = True
        for i, slider_var in enumerate(self._slider_vars):
            slider_var.set(0.0)
            self.target_joints[i] = 0.0
        self._suppress_slider_callbacks = False
        self._pending_send = True

    def _go_servo_ready(self):
        self._suppress_slider_callbacks = True
        for i, slider_var in enumerate(self._slider_vars):
            slider_var.set(math.degrees(SERVO_READY_POSITION[i]))
            self.target_joints[i] = SERVO_READY_POSITION[i]
        self._suppress_slider_callbacks = False
        self._pending_send = True
        self._set_status("Servo-ready pose selected", "#444")

    def _hold_current(self):
        if not self._state_received:
            self._set_status("Waiting for joint states...", "orange")
            return
        self._suppress_slider_callbacks = True
        for i, slider_var in enumerate(self._slider_vars):
            slider_var.set(math.degrees(self.current_joints[i]))
            self.target_joints[i] = self.current_joints[i]
        self._suppress_slider_callbacks = False
        self._pending_send = True

    def _mark_for_send(self):
        self._pending_send = True

    def _periodic_publish(self):
        if self._path_running:
            self.root.after(int(self.command_period * 1000), self._periodic_publish)
            return

        if self._pending_send and self._state_received:
            if self._last_sent_target != self.target_joints:
                self._publish_target(self.target_joints, "Target sent")
                self._last_sent_target = list(self.target_joints)
            self._pending_send = False
        self.root.after(int(self.command_period * 1000), self._periodic_publish)

    def _save_waypoint(self):
        if not self._state_received:
            self._set_status("Waiting for joint states...", "orange")
            return

        waypoint_index = len(self.waypoints) + 1
        joints = list(self.current_joints)
        waypoint_name = f"WP{waypoint_index:02d}"
        self.waypoints.append({"name": waypoint_name, "joints": joints})
        self.waypoint_list.insert(tk.END, f"{waypoint_name}: {self._format_joint_degrees(joints)}")
        self._set_status(f"Saved {waypoint_name}", "green")

    def _delete_waypoint(self):
        selection = self.waypoint_list.curselection()
        if not selection:
            self._set_status("Select a waypoint first", "orange")
            return

        index = int(selection[0])
        self.waypoint_list.delete(index)
        del self.waypoints[index]
        self._set_status("Waypoint deleted", "green")

    def _run_waypoints(self):
        if not self.waypoints:
            self._set_status("No waypoints saved", "orange")
            return

        if self._path_running:
            self._publish_waypoint_trajectory(list(self.waypoints))
            self._set_status("Path already running: trajectory reissued", "orange")
            self.get_logger().warning("Path was already running, reissued smooth trajectory")
            return

        self._path_running = True
        self._pending_send = False

        self._set_status("Running waypoint path...", "orange")
        self.get_logger().info(
            f"Starting waypoint path with {len(self.waypoints)} points "
            f"(tol={math.degrees(self.waypoint_tolerance_rad):.2f} deg, "
            f"timeout={self.waypoint_timeout_sec:.2f} s, "
            f"segment_time={self.waypoint_segment_time:.2f} s)"
        )

        def worker():
            try:
                if not rclpy.ok():
                    return

                waypoints_snapshot = list(self.waypoints)
                for index, waypoint in enumerate(waypoints_snapshot, start=1):
                    self.get_logger().info(
                        f"Waypoint {index}/{len(waypoints_snapshot)}: {waypoint['name']} "
                        f"target={self._format_joint_degrees(waypoint['joints'])}"
                    )

                total_traj_time = self._publish_waypoint_trajectory(waypoints_snapshot)

                final_waypoint = waypoints_snapshot[-1]
                wait_timeout = max(self.waypoint_timeout_sec, total_traj_time + 3.0)
                reached = self._wait_until_reached(
                    final_waypoint["joints"],
                    final_waypoint["name"],
                    allow_reissue=False,
                    timeout_override_sec=wait_timeout,
                )
                if not reached:
                    self._set_status(f"Timeout reaching {final_waypoint['name']}", "red")
                    return

                self._set_status("Waypoint path complete", "green")
                self.get_logger().info("Waypoint path complete")
            finally:
                self._path_running = False

        threading.Thread(target=worker, daemon=True).start()

    def _publish_waypoint_trajectory(self, waypoints):
        message = JointTrajectory()
        message.joint_names = list(self.joint_names)

        elapsed = 0.0
        per_segment_min = max(self.waypoint_segment_time, 0.2)
        max_speed_rad_s = math.radians(max(self.waypoint_max_speed_deg_s, 1.0))
        max_accel_rad_s2 = math.radians(max(self.waypoint_max_accel_deg_s2, 1.0))
        previous = list(self.current_joints)
        times = []
        positions = []

        for waypoint in waypoints:
            delta_max = max(
                self._shortest_angular_distance(cur, tgt)
                for cur, tgt in zip(previous, waypoint["joints"])
            )
            speed_time = delta_max / max_speed_rad_s
            accel_time = 2.0 * math.sqrt(delta_max / max_accel_rad_s2) if delta_max > 0.0 else 0.0
            segment_time = max(per_segment_min, speed_time, accel_time)
            elapsed += segment_time

            point = JointTrajectoryPoint()
            point.positions = list(waypoint["joints"])
            point.time_from_start.sec = int(elapsed)
            point.time_from_start.nanosec = int((elapsed - int(elapsed)) * 1e9)
            message.points.append(point)

            times.append(elapsed)
            positions.append(list(waypoint["joints"]))
            previous = list(waypoint["joints"])

        zero = [0.0] * len(self.joint_names)
        for i, point in enumerate(message.points):
            if i == 0 or i == len(message.points) - 1:
                point.velocities = list(zero)
                point.accelerations = list(zero)
                continue

            dt = times[i + 1] - times[i - 1]
            if dt <= 1e-6:
                point.velocities = list(zero)
            else:
                v = [(positions[i + 1][j] - positions[i - 1][j]) / dt for j in range(len(self.joint_names))]
                point.velocities = [max(min(value, max_speed_rad_s), -max_speed_rad_s) for value in v]

            dt_prev = times[i] - times[i - 1]
            dt_next = times[i + 1] - times[i]
            if dt_prev <= 1e-6 or dt_next <= 1e-6 or (dt_prev + dt_next) <= 1e-6:
                point.accelerations = list(zero)
            else:
                a = [
                    2.0
                    * (
                        (positions[i + 1][j] - positions[i][j]) / dt_next
                        - (positions[i][j] - positions[i - 1][j]) / dt_prev
                    )
                    / (dt_prev + dt_next)
                    for j in range(len(self.joint_names))
                ]
                point.accelerations = [
                    max(min(value, max_accel_rad_s2), -max_accel_rad_s2)
                    for value in a
                ]

        self.publisher.publish(message)
        self._set_status("Smooth path trajectory sent", "green")
        self.get_logger().info(
            f"Published smooth trajectory with {len(message.points)} points "
            f"(min-segment {per_segment_min:.2f}s, max_speed {self.waypoint_max_speed_deg_s:.1f} deg/s, "
            f"max_accel {self.waypoint_max_accel_deg_s2:.1f} deg/s^2, "
            f"total {elapsed:.2f}s)"
        )
        return elapsed

    def _publish_target(self, target_joints, status_text=None):
        message = JointTrajectory()
        message.joint_names = list(self.joint_names)

        point = JointTrajectoryPoint()
        point.positions = list(target_joints)
        point.time_from_start.sec = int(self.trajectory_time)
        point.time_from_start.nanosec = int((self.trajectory_time - int(self.trajectory_time)) * 1e9)
        message.points = [point]

        self.publisher.publish(message)
        if status_text is not None:
            self._set_status(status_text, "green")

    def _wait_until_reached(self, target_joints, waypoint_name, allow_reissue=False, timeout_override_sec=None):
        start_time = time.time()
        timeout_sec = self.waypoint_timeout_sec if timeout_override_sec is None else timeout_override_sec
        deadline = time.time() + max(timeout_sec, 0.5)
        last_log_time = 0.0
        last_reissue_time = start_time
        best_err = float("inf")
        last_progress_time = start_time

        while rclpy.ok() and time.time() < deadline:
            errs = [
                self._shortest_angular_distance(cur, tgt)
                for cur, tgt in zip(self.current_joints, target_joints)
            ]
            max_err = max(errs)

            now = time.time()
            if max_err + math.radians(0.1) < best_err:
                best_err = max_err
                last_progress_time = now

            if now - last_log_time >= max(self.waypoint_log_period_sec, 0.1):
                elapsed = now - start_time
                err_deg = [f"{math.degrees(value):.2f}" for value in errs]
                self.get_logger().info(
                    f"Waiting {waypoint_name}: t={elapsed:.2f}s "
                    f"max_err={math.degrees(max_err):.2f} deg errs_deg={err_deg}"
                )
                last_log_time = now

            if allow_reissue and max_err > self.waypoint_tolerance_rad and (
                now - last_reissue_time >= max(self.waypoint_reissue_period_sec, 0.2)
            ):
                self._publish_target(target_joints)
                self.get_logger().info(
                    f"Reissued target for {waypoint_name} "
                    f"(max_err={math.degrees(max_err):.2f} deg)"
                )
                last_reissue_time = now

            if max_err <= self.waypoint_tolerance_rad:
                self.get_logger().info(
                    f"Reached {waypoint_name} in {time.time() - start_time:.2f}s "
                    f"(max_err={math.degrees(max_err):.2f} deg)"
                )
                return True

            if now - last_progress_time >= max(self.waypoint_stall_timeout_sec, 0.5):
                self.get_logger().warning(
                    f"Stall detected for {waypoint_name}: no progress for "
                    f"{now - last_progress_time:.2f}s (max_err={math.degrees(max_err):.2f} deg)"
                )
                return False

            time.sleep(0.05)

        final_errs = [
            self._shortest_angular_distance(cur, tgt)
            for cur, tgt in zip(self.current_joints, target_joints)
        ]
        final_max = max(final_errs)
        final_err_deg = [f"{math.degrees(value):.2f}" for value in final_errs]
        self.get_logger().warning(
            f"Timeout reaching {waypoint_name} after {timeout_sec:.2f}s "
            f"(max_err={math.degrees(final_max):.2f} deg, errs_deg={final_err_deg}, "
            f"current={self._format_joint_degrees(self.current_joints)}, "
            f"target={self._format_joint_degrees(target_joints)})"
        )
        return False

    def _shortest_angular_distance(self, current, target):
        # Wrap difference to [-pi, pi] so equivalent joint angles don't cause false timeout.
        return abs(math.atan2(math.sin(target - current), math.cos(target - current)))

    def _format_joint_degrees(self, joints):
        return "[" + ", ".join(f"{math.degrees(value):.1f} deg" for value in joints) + "]"

    def _set_status(self, text, color):
        self.root.after(0, lambda: self.status.config(text=text, fg=color))

    def run(self):
        self.root.mainloop()

    def destroy(self):
        self._stop_hold_jog()
        self._stop_hold_cartesian()
        self.root.destroy()
        super().destroy_node()


def main():
    rclpy.init()
    gui = TeleopGUI()

    executor = MultiThreadedExecutor()
    executor.add_node(gui)
    ros_thread = threading.Thread(target=executor.spin, daemon=True)
    ros_thread.start()

    try:
        gui.run()
    except KeyboardInterrupt:
        pass
    finally:
        gui.destroy()
        executor.shutdown()
        rclpy.shutdown()


if __name__ == "__main__":
    main()
