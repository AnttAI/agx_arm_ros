"""Drive a Tara differential-drive base through native ROS 2 interfaces."""

from __future__ import annotations

import csv
import io
import json
import math
import threading
import time
from typing import Any

import rclpy
from geometry_msgs.msg import Twist
from rclpy.node import Node
from std_msgs.msg import Bool, Float64MultiArray, String
from std_srvs.srv import SetBool, Trigger


TARA_ENCODER_COUNTS_PER_WHEEL_REVOLUTION = 4096
ENCODER_TURN_TIMEOUT_S = 8.0


class TaraBaseNode(Node):
    """Translate ROS velocity commands to Tara motor RPM commands."""

    def __init__(self) -> None:
        super().__init__("tara_base")

        self.declare_parameter("serial_port", "/dev/ttyUSB0")
        self.declare_parameter("slave_id", 1)
        self.declare_parameter("baudrate", 115200)
        self.declare_parameter("wheel_radius_m", 0.10)
        self.declare_parameter("wheel_separation_m", 0.157)
        self.declare_parameter("max_abs_rpm", 30.0)
        self.declare_parameter("command_timeout_s", 0.5)
        self.declare_parameter("feedback_rate_hz", 5.0)
        self.declare_parameter("left_motor_sign", 1)
        self.declare_parameter("right_motor_sign", -1)
        self.declare_parameter("debug", False)

        self._wheel_radius = float(self.get_parameter("wheel_radius_m").value)
        self._wheel_separation = float(self.get_parameter("wheel_separation_m").value)
        self._max_abs_rpm = float(self.get_parameter("max_abs_rpm").value)
        self._timeout = float(self.get_parameter("command_timeout_s").value)
        self._left_sign = int(self.get_parameter("left_motor_sign").value)
        self._right_sign = int(self.get_parameter("right_motor_sign").value)
        feedback_rate = float(self.get_parameter("feedback_rate_hz").value)
        self._validate_parameters(feedback_rate)

        self._lock = threading.RLock()
        self._base: Any | None = None
        self._enabled = False
        self._last_command_time = time.monotonic()
        self._watchdog_stopped = True
        self._csv_stop_event = threading.Event()
        self._csv_thread: threading.Thread | None = None
        self._csv_running = False

        self._connected_pub = self.create_publisher(Bool, "connected", 1)
        self._wheel_rpm_pub = self.create_publisher(Float64MultiArray, "wheel_rpm", 10)
        self._csv_status_pub = self.create_publisher(String, "csv_status", 10)
        self.create_subscription(Twist, "cmd_vel", self._cmd_vel_callback, 10)
        self.create_subscription(
            Float64MultiArray, "cmd_wheel_rpm", self._cmd_wheel_rpm_callback, 10
        )
        self.create_subscription(String, "play_csv", self._play_csv_callback, 10)
        self.create_service(SetBool, "enable", self._enable_callback)
        self.create_service(Trigger, "stop", self._stop_callback)
        self.create_service(Trigger, "emergency_stop", self._emergency_stop_callback)
        self.create_service(Trigger, "clear_fault", self._clear_fault_callback)
        self.create_service(Trigger, "reconnect", self._reconnect_callback)

        self.create_timer(min(0.05, self._timeout / 2.0), self._watchdog_callback)
        self.create_timer(1.0 / feedback_rate, self._feedback_callback)
        self._connect()

    def _validate_parameters(self, feedback_rate: float) -> None:
        if self._wheel_radius <= 0.0 or self._wheel_separation <= 0.0:
            raise ValueError("wheel dimensions must be positive")
        if not 0.0 < self._max_abs_rpm <= 3000.0:
            raise ValueError("max_abs_rpm must be in (0, 3000]")
        if self._timeout <= 0.0 or feedback_rate <= 0.0:
            raise ValueError("command timeout and feedback rate must be positive")
        if self._left_sign not in (-1, 1) or self._right_sign not in (-1, 1):
            raise ValueError("motor signs must be either -1 or 1")

    def _connect(self) -> bool:
        with self._lock:
            self._disconnect()
            base = None
            try:
                from tara_base_ctrl.vendor.tara_sdk import TaraBase

                base = TaraBase(
                    port=str(self.get_parameter("serial_port").value),
                    slave_id=int(self.get_parameter("slave_id").value),
                    baudrate=int(self.get_parameter("baudrate").value),
                    debug=bool(self.get_parameter("debug").value),
                )
                base.connect()
                if not base.connected:
                    raise RuntimeError("the Tara SDK could not connect")
                if not base.clear_fault():
                    raise RuntimeError("could not clear Tara base faults")
                # Keep the wheels torque-free until the first nonzero command.
                if not base.stop_motors():
                    raise RuntimeError("could not disable the Tara base motors")
                self._base = base
                self._enabled = False
                self._last_command_time = time.monotonic()
                self._watchdog_stopped = True
                self.get_logger().info(
                    f"Connected to Tara base on {self.get_parameter('serial_port').value}; "
                    "motors disabled until the first RPM command"
                )
                self._publish_connected()
                return True
            except Exception as exc:  # Hardware/import errors should leave ROS alive.
                if base is not None and base.connected:
                    base.disconnect()
                self._base = None
                self._enabled = False
                self.get_logger().error(f"Tara base connection failed: {exc}")
                self._publish_connected()
                return False

    def _disconnect(self) -> None:
        if self._base is not None:
            try:
                self._base.disconnect()
            except Exception as exc:
                self.get_logger().warning(f"Tara base disconnect failed: {exc}")
        self._base = None
        self._enabled = False

    def _publish_connected(self) -> None:
        msg = Bool()
        msg.data = self._base is not None and bool(self._base.connected)
        self._connected_pub.publish(msg)

    def _cmd_vel_callback(self, msg: Twist) -> None:
        if self._csv_running:
            self.get_logger().warning("Ignoring cmd_vel while CSV playback is active")
            return
        linear = float(msg.linear.x)
        angular = float(msg.angular.z)
        left_m_s = linear - angular * self._wheel_separation / 2.0
        right_m_s = linear + angular * self._wheel_separation / 2.0
        rpm_factor = 60.0 / (2.0 * math.pi * self._wheel_radius)
        self._set_logical_wheel_rpm(left_m_s * rpm_factor, right_m_s * rpm_factor)

    def _cmd_wheel_rpm_callback(self, msg: Float64MultiArray) -> None:
        if self._csv_running:
            self.get_logger().warning("Ignoring cmd_wheel_rpm while CSV playback is active")
            return
        if len(msg.data) != 2:
            self.get_logger().warning("cmd_wheel_rpm requires [left_rpm, right_rpm]")
            return
        self._set_logical_wheel_rpm(float(msg.data[0]), float(msg.data[1]))

    def _set_logical_wheel_rpm(self, left_rpm: float, right_rpm: float) -> bool:
        if not math.isfinite(left_rpm) or not math.isfinite(right_rpm):
            self.get_logger().warning("Ignoring non-finite wheel command")
            return False

        peak = max(abs(left_rpm), abs(right_rpm))
        if peak > self._max_abs_rpm:
            scale = self._max_abs_rpm / peak
            left_rpm *= scale
            right_rpm *= scale

        with self._lock:
            if self._base is None:
                self.get_logger().warning("Ignoring command: Tara base is not connected")
                return False
            motor_left = int(round(left_rpm * self._left_sign))
            motor_right = int(round(right_rpm * self._right_sign))

            # Zero is a valid explicit CSV/topic command. It holds the base
            # stopped until commands cease and the watchdog releases torque.
            if not self._enabled:
                if not self._base.enable_motors():
                    self.get_logger().error("Could not enable the Tara base motors")
                    return False
                self._enabled = True

            if not self._base.set_velocity(motor_left, motor_right):
                self.get_logger().error("Tara set_velocity failed; stopping the motors")
                self._base.stop_motors()
                self._enabled = False
                self._watchdog_stopped = True
                return False
            self._last_command_time = time.monotonic()
            self._watchdog_stopped = False
            return True

    def _publish_csv_status(self, **values: Any) -> None:
        payload = {
            "running": self._csv_running,
            "message": "",
            "index": 0,
            "total": 0,
            **values,
        }
        msg = String()
        msg.data = json.dumps(payload, separators=(",", ":"))
        self._csv_status_pub.publish(msg)
        if payload["message"] and not str(payload["message"]).startswith("Sending row "):
            self.get_logger().info(str(payload["message"]))

    @staticmethod
    def _load_csv_commands(csv_text: str) -> list[dict[str, Any]]:
        reader = csv.DictReader(io.StringIO(csv_text))
        if reader.fieldnames is None:
            raise ValueError("CSV has no header")
        left_column = (
            "left_motor_rpm" if "left_motor_rpm" in reader.fieldnames else "left_wheel_rpm"
        )
        right_column = (
            "right_motor_rpm" if "right_motor_rpm" in reader.fieldnames else "right_wheel_rpm"
        )
        missing = [name for name in (left_column, right_column) if name not in reader.fieldnames]
        if missing:
            raise ValueError(
                "CSV is missing " + ", ".join(missing)
                + "; expected left_motor_rpm/right_motor_rpm"
            )

        commands: list[dict[str, Any]] = []
        for row_index, row in enumerate(reader):
            target_text = str(row.get("encoder_target_wheel_revolutions", "")).strip()
            commands.append(
                {
                    "frame": int(float(row.get("Frame", row_index))),
                    "left_rpm": float(row[left_column]),
                    "right_rpm": float(row[right_column]),
                    "action": str(row.get("base_action", "")).strip(),
                    "encoder_target": float(target_text) if target_text else None,
                }
            )
        if not commands:
            raise ValueError("CSV has no command rows")
        return commands

    def _play_csv_callback(self, msg: String) -> None:
        try:
            payload = json.loads(msg.data)
            if not isinstance(payload, dict):
                raise TypeError("play_csv payload must be a JSON object")
            if str(payload.get("action", "")).strip().lower() == "stop":
                self._csv_stop_event.set()
                self._publish_csv_status(message="ROS TaraBase CSV stop requested.")
                return

            commands = self._load_csv_commands(str(payload["csv_text"]))
            options = dict(payload.get("options", {}))
            with self._lock:
                if self._csv_running:
                    raise RuntimeError("TaraBase CSV playback is already running")
                self._csv_running = True
                self._csv_stop_event.clear()
            self._csv_thread = threading.Thread(
                target=self._run_csv_stream,
                args=(commands, options),
                name="tara-ros-csv-playback",
                daemon=True,
            )
            self._csv_thread.start()
        except (json.JSONDecodeError, KeyError, TypeError, ValueError, RuntimeError) as exc:
            self._publish_csv_status(message=f"TaraBase CSV request failed: {exc}")

    def _encoder_positions(self) -> tuple[int, int]:
        with self._lock:
            if self._base is None:
                raise RuntimeError("Tara base is not connected")
            positions = self._base.get_motor_positions()
        if positions is None:
            raise RuntimeError("Could not read TaraBase encoder positions")
        return (
            int(positions["left_motor_position"]),
            int(positions["right_motor_position"]),
        )

    @staticmethod
    def _signed_32_bit_delta(current: int, start: int) -> int:
        return ((current - start + 0x80000000) % 0x100000000) - 0x80000000

    def _encoder_revolutions(self, start: tuple[int, int]) -> float:
        current = self._encoder_positions()
        left_counts = abs(self._signed_32_bit_delta(current[0], start[0]))
        right_counts = abs(self._signed_32_bit_delta(current[1], start[1]))
        return 0.5 * (left_counts + right_counts) / TARA_ENCODER_COUNTS_PER_WHEEL_REVOLUTION

    def _stop_csv_hardware(self) -> None:
        with self._lock:
            if self._base is not None:
                self._base.stop_motors()
            self._enabled = False
            self._watchdog_stopped = True

    def _prepare_csv_hardware(self) -> None:
        """Reset the motor controller exactly as a fresh WebSocket stream does."""
        with self._lock:
            if self._base is None or not self._base.connected:
                raise RuntimeError("Tara base is not connected")
            # TaraBase.connect() calls this for every WebSocket stream. The ROS
            # node keeps its serial connection open, so repeat it explicitly.
            self._base._initialize_velocity_control()
            if not self._base.clear_fault():
                raise RuntimeError("Could not clear TaraBase faults")
            if not self._base.enable_motors():
                raise RuntimeError("Could not enable TaraBase motors")
            self._enabled = True
            self._watchdog_stopped = True

    def _run_csv_stream(self, commands: list[dict[str, Any]], options: dict[str, Any]) -> None:
        try:
            fps = float(options.get("fps", 30.0))
            speed_scale = float(options.get("speed_scale", 1.0))
            max_abs_rpm = min(float(options.get("max_abs_rpm", 30.0)), self._max_abs_rpm)
            start_index = int(options.get("start_index", 0))
            reverse_playback = bool(options.get("reverse_playback", False))
            fit_to_limit = bool(options.get("fit_to_rpm_limit", True))
            if fps <= 0.0 or speed_scale <= 0.0 or max_abs_rpm <= 0.0:
                raise ValueError("fps, speed_scale, and max_abs_rpm must be positive")

            if reverse_playback:
                commands = [
                    {
                        **command,
                        "left_rpm": -float(command["left_rpm"]),
                        "right_rpm": -float(command["right_rpm"]),
                        "encoder_target": None,
                    }
                    for command in reversed(commands)
                ]
            if start_index < 0 or start_index >= len(commands):
                raise ValueError(f"start_index {start_index} is outside {len(commands)} CSV rows")

            self._prepare_csv_hardware()

            effective_playback_speed = 1.0
            peak = max(
                max(abs(float(command["left_rpm"])), abs(float(command["right_rpm"])))
                * speed_scale
                for command in commands
            )
            if fit_to_limit and peak > max_abs_rpm:
                effective_playback_speed = max_abs_rpm / peak
            dt = 1.0 / (fps * effective_playback_speed)
            total = len(commands)
            self._publish_csv_status(
                running=True,
                message=(
                    f"ROS TaraBase streaming row {start_index + 1}/{total} at {fps:g} FPS, "
                    f"RPM scale {speed_scale:g}, clamp +/-{max_abs_rpm:g} RPM."
                ),
                index=start_index,
                total=total,
            )

            encoder_start: tuple[int, int] | None = None
            encoder_target: float | None = None
            encoder_started_at: float | None = None
            encoder_drive_rpm: tuple[float, float] | None = None
            encoder_action: str | None = None
            encoder_revolutions = 0.0
            stream_start = time.monotonic()
            last_progress_publish_time = 0.0

            for index in range(start_index, total):
                if self._csv_stop_event.is_set():
                    break
                command = commands[index]
                csv_left = float(command["left_rpm"])
                csv_right = float(command["right_rpm"])
                action = str(command["action"])
                target = command["encoder_target"]
                encoder_controlled = (
                    not reverse_playback
                    and action in {"turn_left_90", "turn_right_90"}
                    and target is not None
                )

                if encoder_controlled:
                    if encoder_action != action:
                        encoder_start = self._encoder_positions()
                        encoder_target = float(target)
                        encoder_started_at = time.monotonic()
                        encoder_drive_rpm = None
                        encoder_action = action
                        encoder_revolutions = 0.0
                    else:
                        assert encoder_start is not None
                        encoder_revolutions = self._encoder_revolutions(encoder_start)
                    assert encoder_target is not None
                    if encoder_revolutions >= encoder_target:
                        csv_left = 0.0
                        csv_right = 0.0
                    else:
                        if csv_left != 0.0 or csv_right != 0.0:
                            encoder_drive_rpm = (csv_left, csv_right)
                        elif encoder_drive_rpm is not None:
                            csv_left, csv_right = encoder_drive_rpm
                        assert encoder_started_at is not None
                        if time.monotonic() - encoder_started_at > ENCODER_TURN_TIMEOUT_S:
                            raise RuntimeError(
                                f"{action} encoder target was not reached within "
                                f"{ENCODER_TURN_TIMEOUT_S:.1f}s"
                            )

                scaled_left = max(
                    -max_abs_rpm,
                    min(max_abs_rpm, csv_left * speed_scale * effective_playback_speed),
                )
                scaled_right = max(
                    -max_abs_rpm,
                    min(max_abs_rpm, csv_right * speed_scale * effective_playback_speed),
                )
                # Exactly match stream_wheel_commands(invert_turn_direction=True):
                # swap logical wheels, then _set_logical_wheel_rpm applies the
                # physical motor signs (+1 left, -1 right).
                if not self._set_logical_wheel_rpm(scaled_right, scaled_left):
                    raise RuntimeError(f"set_velocity failed at frame {command['frame']}")
                now = time.monotonic()
                if now - last_progress_publish_time >= 0.2 or index + 1 == total:
                    self._publish_csv_status(
                        running=True,
                        message=f"Sending row {index + 1}/{total}.",
                        index=index + 1,
                        total=total,
                    )
                    last_progress_publish_time = now

                target_time = stream_start + ((index - start_index + 1) * dt)
                wait_seconds = target_time - time.monotonic()
                if wait_seconds > 0.0 and self._csv_stop_event.wait(wait_seconds):
                    break

            while (
                not self._csv_stop_event.is_set()
                and encoder_start is not None
                and encoder_target is not None
                and encoder_drive_rpm is not None
                and encoder_revolutions < encoder_target
            ):
                encoder_revolutions = self._encoder_revolutions(encoder_start)
                if encoder_revolutions >= encoder_target:
                    break
                assert encoder_started_at is not None
                if time.monotonic() - encoder_started_at > ENCODER_TURN_TIMEOUT_S:
                    raise RuntimeError(
                        "90-degree encoder target was not reached within "
                        f"{ENCODER_TURN_TIMEOUT_S:.1f}s"
                    )
                # Match the WebSocket sender exactly: the motor controller
                # retains the last turn command here. Only poll encoders; do
                # not add extra set_velocity Modbus writes at 50 Hz.
                self._csv_stop_event.wait(0.02)

            if encoder_target is not None and encoder_revolutions >= encoder_target:
                if not self._set_logical_wheel_rpm(0.0, 0.0):
                    raise RuntimeError("Could not send zero RPM at encoder target")

            final_message = (
                "ROS TaraBase CSV stopped."
                if self._csv_stop_event.is_set()
                else "ROS TaraBase CSV finished."
            )
            if encoder_target is not None and encoder_revolutions >= encoder_target:
                final_message += f" Encoder target reached at {encoder_revolutions:.3f} wheel rev."
            self._publish_csv_status(
                running=False,
                message=final_message,
                index=total,
                total=total,
            )
        except Exception as exc:
            self._publish_csv_status(
                running=False, message=f"TaraBase ROS CSV playback failed: {exc}"
            )
        finally:
            self._stop_csv_hardware()
            with self._lock:
                self._csv_running = False
            self._csv_thread = None

    def _watchdog_callback(self) -> None:
        with self._lock:
            if self._base is None or self._watchdog_stopped:
                return
            if time.monotonic() - self._last_command_time >= self._timeout:
                self._base.stop_motors()
                self._enabled = False
                self._watchdog_stopped = True
                self.get_logger().warning(
                    "Command watchdog disabled the Tara base motors (freewheel)"
                )

    def _feedback_callback(self) -> None:
        # Match the direct/WebSocket CSV sender: do not insert extra Modbus
        # reads between timed wheel commands. Each velocity feedback cycle
        # performs two serial transactions and can otherwise delay playback.
        if self._csv_running:
            return
        with self._lock:
            if self._base is None:
                self._publish_connected()
                return
            velocities = self._base.get_actual_velocity()
            if velocities is None:
                self.get_logger().warning("Could not read Tara wheel velocity feedback")
                return
            msg = Float64MultiArray()
            msg.data = [
                float(velocities["left_motor_velocity"]) * self._left_sign,
                float(velocities["right_motor_velocity"]) * self._right_sign,
            ]
            self._wheel_rpm_pub.publish(msg)
            self._publish_connected()

    def _enable_callback(self, request: SetBool.Request, response: SetBool.Response):
        with self._lock:
            if self._base is None:
                response.success = False
                response.message = "Tara base is not connected"
            elif request.data:
                response.success = bool(self._base.enable_motors())
                self._enabled = response.success
                response.message = "Motors enabled" if response.success else "Enable failed"
            else:
                response.success = bool(self._base.stop_motors())
                self._enabled = False
                self._watchdog_stopped = True
                response.message = "Motors disabled" if response.success else "Stop failed"
        return response

    def _stop_callback(self, _request: Trigger.Request, response: Trigger.Response):
        self._csv_stop_event.set()
        with self._lock:
            response.success = self._base is not None and bool(self._base.stop_motors())
            self._enabled = False
            self._watchdog_stopped = True
            response.message = "Motors stopped" if response.success else "Stop failed"
        return response

    def _emergency_stop_callback(
        self, _request: Trigger.Request, response: Trigger.Response
    ):
        self._csv_stop_event.set()
        with self._lock:
            response.success = self._base is not None and bool(self._base.emergency_stop())
            self._enabled = False
            self._watchdog_stopped = True
            response.message = (
                "Emergency stop activated" if response.success else "Emergency stop failed"
            )
        return response

    def _clear_fault_callback(
        self, _request: Trigger.Request, response: Trigger.Response
    ):
        with self._lock:
            response.success = self._base is not None and bool(self._base.clear_fault())
            response.message = "Fault cleared" if response.success else "Clear fault failed"
        return response

    def _reconnect_callback(self, _request: Trigger.Request, response: Trigger.Response):
        response.success = self._connect()
        response.message = "Connected" if response.success else "Connection failed; see node log"
        return response

    def destroy_node(self) -> bool:
        self._csv_stop_event.set()
        csv_thread = self._csv_thread
        if csv_thread is not None and csv_thread.is_alive():
            csv_thread.join(timeout=1.0)
        with self._lock:
            self._disconnect()
        return super().destroy_node()


def main(args=None) -> None:
    rclpy.init(args=args)
    node = TaraBaseNode()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == "__main__":
    main()
