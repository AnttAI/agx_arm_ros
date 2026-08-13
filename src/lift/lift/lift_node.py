#!/usr/bin/env python3
from __future__ import annotations

import math
import threading
import time
from types import SimpleNamespace

import rclpy
from rclpy.node import Node
from std_msgs.msg import Bool, Float64, Float64MultiArray, String
from std_srvs.srv import Trigger

try:
    import serial
except ImportError:  # Keep import errors visible when the node starts.
    serial = None


WAKE = bytes.fromhex("9b 06 02 00 00 6c a1 9d")
UP = bytes.fromhex("9b 06 02 01 00 fc a0 9d")
DOWN = bytes.fromhex("9b 06 02 02 00 0c a0 9d")

SEGMENT_DIGITS = {
    0x3F: "0",
    0x06: "1",
    0x5B: "2",
    0x4F: "3",
    0x66: "4",
    0x6D: "5",
    0x7D: "6",
    0x07: "7",
    0x7F: "8",
    0x6F: "9",
}


def decode_height(frame: bytes) -> float | None:
    if len(frame) != 9 or frame[2] != 0x12:
        return None

    chars: list[str] = []
    decimal_after: int | None = None
    for raw in frame[3:6]:
        digit = SEGMENT_DIGITS.get(raw & 0x7F)
        if digit is None:
            return None
        chars.append(digit)
        if raw & 0x80:
            decimal_after = len(chars)

    value = "".join(chars)
    if decimal_after is not None and decimal_after < len(value):
        value = f"{value[:decimal_after]}.{value[decimal_after:]}"
    return float(value)


def pop_heights(buffer: bytearray) -> list[float]:
    heights: list[float] = []
    while True:
        try:
            start = buffer.index(0x9B)
        except ValueError:
            buffer.clear()
            break
        if start:
            del buffer[:start]
        try:
            end = buffer.index(0x9D, 1)
        except ValueError:
            break

        frame = bytes(buffer[: end + 1])
        del buffer[: end + 1]
        expected_size = frame[1] + 2 if len(frame) >= 2 else 0
        if expected_size and len(frame) != expected_size:
            continue
        height = decode_height(frame)
        if height is not None:
            heights.append(height)
    return heights


class LiftNode(Node):
    def __init__(self) -> None:
        super().__init__("lift_node")

        self._declare_parameters()
        self._load_parameters()

        self._lock = threading.RLock()
        self._buffer = bytearray()
        self._serial = None
        self._connected = False
        self._last_height: float | None = None
        self._target_lock = threading.Lock()
        self._latest_target: tuple[int | None, float] | None = None
        self._target_event = threading.Event()
        self._retarget_event = threading.Event()
        self._stop_event = threading.Event()
        self._shutdown_event = threading.Event()

        self._height_pub = self.create_publisher(Float64, "/feedback/lift_height", 10)
        self._status_pub = self.create_publisher(String, "/feedback/lift_status", 10)
        self._connected_pub = self.create_publisher(Bool, "/feedback/lift_connected", 1)

        self.create_subscription(Float64, "/control/lift_height", self._height_callback, 10)
        self.create_subscription(
            Float64MultiArray, "/control/lift_frame", self._frame_callback, 100
        )
        self.create_service(Trigger, "/lift_stop", self._stop_callback)
        self.create_service(Trigger, "/lift_reconnect", self._reconnect_callback)

        self.create_timer(1.0, self._publish_connected)
        self._connect()

        self._worker = threading.Thread(target=self._worker_loop, name="lift-target-worker")
        self._worker.start()

    def _declare_parameters(self) -> None:
        self.declare_parameter("serial_port", "")
        self.declare_parameter("min_height", 60.0)
        self.declare_parameter("max_height", 120.0)
        self.declare_parameter("tolerance", 1.0)
        self.declare_parameter("interval", 0.108)
        self.declare_parameter("max_run_seconds", 25.0)
        self.declare_parameter("max_attempts", 2)
        self.declare_parameter("settle_seconds", 1.8)
        self.declare_parameter("min_command_seconds", 0.08)
        self.declare_parameter("max_command_seconds", 10.0)
        self.declare_parameter("brake_timeout_margin", 2.0)
        self.declare_parameter("active_brake_packets", 1)
        self.declare_parameter("up_speed", 4.4)
        self.declare_parameter("down_speed", 5.8)
        self.declare_parameter("up_brake_distance", 5.8)
        self.declare_parameter("down_brake_distance", 3.8)
        self.declare_parameter("wake_timeout_seconds", 30.0)

    def _load_parameters(self) -> None:
        self.serial_port = str(self.get_parameter("serial_port").value)
        self.min_height = float(self.get_parameter("min_height").value)
        self.max_height = float(self.get_parameter("max_height").value)
        self.tolerance = float(self.get_parameter("tolerance").value)
        self.interval = float(self.get_parameter("interval").value)
        self.max_run_seconds = float(self.get_parameter("max_run_seconds").value)
        self.wake_timeout_seconds = float(self.get_parameter("wake_timeout_seconds").value)
        self.speed_estimates = {
            "up": float(self.get_parameter("up_speed").value),
            "down": float(self.get_parameter("down_speed").value),
        }
        self.motion_options = SimpleNamespace(
            max_attempts=int(self.get_parameter("max_attempts").value),
            settle_seconds=float(self.get_parameter("settle_seconds").value),
            min_command_seconds=float(self.get_parameter("min_command_seconds").value),
            max_command_seconds=float(self.get_parameter("max_command_seconds").value),
            brake_timeout_margin=float(self.get_parameter("brake_timeout_margin").value),
            active_brake_packets=int(self.get_parameter("active_brake_packets").value),
            up_brake_distance=float(self.get_parameter("up_brake_distance").value),
            down_brake_distance=float(self.get_parameter("down_brake_distance").value),
        )

    def _connect(self) -> bool:
        with self._lock:
            self._disconnect_locked()
            if serial is None:
                self._publish_status("pyserial is missing; install python3-serial")
                return False
            if not self.serial_port:
                self._publish_status("serial_port is required; pass serial_port:=/dev/ttyUSBx")
                return False
            try:
                self._serial = serial.Serial(
                    self.serial_port,
                    baudrate=9600,
                    bytesize=serial.EIGHTBITS,
                    parity=serial.PARITY_NONE,
                    stopbits=serial.STOPBITS_ONE,
                    timeout=0.02,
                    write_timeout=0.2,
                    xonxoff=False,
                    rtscts=False,
                    dsrdtr=False,
                )
                self._serial.reset_input_buffer()
                self._connected = True
                self._buffer.clear()
                self._publish_status(f"Connected to lift on {self.serial_port}")
                self._publish_connected()
                return True
            except Exception as exc:
                self._connected = False
                self._publish_status(f"Lift connection failed on {self.serial_port}: {exc}")
                self._publish_connected()
                return False

    def _disconnect_locked(self) -> None:
        if self._serial is not None:
            try:
                self._serial.close()
            except Exception as exc:
                self.get_logger().warning(f"Lift serial close failed: {exc}")
        self._serial = None
        self._connected = False

    def _publish_status(self, text: str) -> None:
        msg = String()
        msg.data = text
        self._status_pub.publish(msg)
        self.get_logger().info(text)

    def _publish_connected(self) -> None:
        msg = Bool()
        msg.data = self._connected
        self._connected_pub.publish(msg)

    def _publish_height(self, height: float) -> None:
        self._last_height = height
        msg = Float64()
        msg.data = height
        self._height_pub.publish(msg)

    def _height_callback(self, msg: Float64) -> None:
        self._set_latest_target(None, float(msg.data))

    def _frame_callback(self, msg: Float64MultiArray) -> None:
        if len(msg.data) < 2:
            self.get_logger().warning("/control/lift_frame expects [frame_index, height_cm]")
            return
        self._set_latest_target(int(msg.data[0]), float(msg.data[1]))

    def _set_latest_target(self, frame: int | None, target: float) -> None:
        if not math.isfinite(target):
            self.get_logger().warning("Ignoring non-finite lift target")
            return
        if not self.min_height <= target <= self.max_height:
            self.get_logger().warning(
                f"Ignoring lift target {target:.1f}; allowed range is "
                f"{self.min_height:.1f}-{self.max_height:.1f} cm"
            )
            return
        with self._target_lock:
            self._latest_target = (frame, target)
            self._target_event.set()
            self._retarget_event.set()
            self._stop_event.clear()
        label = f"frame {frame}" if frame is not None else "direct target"
        self._publish_status(f"Latest lift {label}: {target:.1f} cm")

    def _worker_loop(self) -> None:
        while not self._shutdown_event.is_set():
            if not self._target_event.wait(timeout=0.1):
                continue

            with self._target_lock:
                latest = self._latest_target
                self._latest_target = None
                self._target_event.clear()
                self._retarget_event.clear()
            if latest is None:
                continue

            frame, target = latest
            label = f"frame {frame}" if frame is not None else None
            try:
                self._run_target(target, label)
            except Exception as exc:
                self._publish_status(f"Lift target failed: {exc}")
                self._connect()

    def _run_target(self, target: float, label: str | None) -> None:
        prefix = f"{label}: " if label else ""
        with self._lock:
            ser = self._serial
            if ser is None or not self._connected:
                raise RuntimeError("lift is not connected")
            current = self._last_height or self._ensure_awake_locked()
            if current is None:
                raise RuntimeError("could not read current lift height")

        self._publish_status(f"{prefix}moving lift from {current:.1f} cm to {target:.1f} cm")
        result, final_height = self._move_to_target(current, target, label)
        if result == 0:
            self._publish_status(f"{prefix}reached {final_height:.1f} cm")
        elif self._stop_event.is_set():
            self._publish_status(f"{prefix}stopped at {final_height:.1f} cm")
        elif result == 9:
            self._publish_status(f"{prefix}retargeting from {final_height:.1f} cm")
        else:
            self._publish_status(f"{prefix}ended at {final_height:.1f} cm before target")

    def _ensure_awake_locked(self) -> float | None:
        ser = self._serial
        if ser is None:
            return None
        self._send_repeated_locked(WAKE, 0.8)
        deadline = time.monotonic() + self.wake_timeout_seconds
        next_wake_at = time.monotonic() + 5.0
        while time.monotonic() < deadline and not self._stop_event.is_set():
            height = self._read_height_locked(1.0)
            if height is not None:
                return height
            if time.monotonic() >= next_wake_at:
                self._send_repeated_locked(WAKE, 0.8)
                next_wake_at = time.monotonic() + 5.0
        return None

    def _read_height_locked(self, seconds: float) -> float | None:
        ser = self._serial
        if ser is None:
            return None
        last_height = None
        deadline = time.monotonic() + seconds
        while time.monotonic() < deadline and not self._stop_event.is_set():
            chunk = ser.read(64)
            if not chunk:
                continue
            self._buffer.extend(chunk)
            for height in pop_heights(self._buffer):
                last_height = height
                self._publish_height(height)
        return last_height

    def _send_repeated_locked(self, packet: bytes, seconds: float) -> int:
        ser = self._serial
        if ser is None:
            return 0
        sent = 0
        deadline = time.monotonic() + seconds
        while time.monotonic() < deadline and not self._stop_event.is_set():
            ser.write(packet)
            ser.flush()
            sent += 1
            time.sleep(self.interval)
        return sent

    def _move_to_target(self, current: float, target: float, label: str | None) -> tuple[int, float]:
        started_at = time.monotonic()
        prefix = f"{label}: " if label else ""
        opts = self.motion_options

        for attempt in range(1, opts.max_attempts + 1):
            if self._stop_event.is_set():
                return 8, current
            if self._retarget_event.is_set():
                return 9, self._last_height if self._last_height is not None else current
            error = target - current
            if abs(error) <= self.tolerance:
                return 0, current
            if time.monotonic() - started_at >= self.max_run_seconds:
                break

            direction = "up" if error > 0 else "down"
            packet = UP if direction == "up" else DOWN
            speed = max(0.1, self.speed_estimates[direction])
            brake_distance = (
                opts.up_brake_distance if direction == "up" else opts.down_brake_distance
            )
            command_seconds = min(
                opts.max_command_seconds,
                max(opts.min_command_seconds, max(0.0, abs(error) - brake_distance) / speed),
            )
            timeout_seconds = min(
                opts.max_command_seconds, command_seconds + opts.brake_timeout_margin
            )
            self._publish_status(
                f"{prefix}attempt {attempt}/{opts.max_attempts}: {direction} "
                f"{current:.1f}->{target:.1f} cm"
            )
            live_height, command_elapsed = self._send_until_brake_point(
                packet, direction, target, brake_distance, timeout_seconds
            )
            if self._retarget_event.is_set():
                return 9, live_height if live_height is not None else current
            next_height = self._settle_and_measure(direction, target)
            if self._retarget_event.is_set():
                return 9, self._last_height if self._last_height is not None else current
            if next_height is None:
                return 4, current

            travelled = abs(next_height - current)
            if live_height is not None:
                measured_brake = abs(next_height - live_height)
                if measured_brake > 0.1:
                    if direction == "up":
                        opts.up_brake_distance = 0.5 * opts.up_brake_distance + 0.5 * measured_brake
                    else:
                        opts.down_brake_distance = 0.5 * opts.down_brake_distance + 0.5 * measured_brake
            if command_elapsed > opts.min_command_seconds and travelled > brake_distance:
                measured_speed = (travelled - brake_distance) / command_elapsed
                self.speed_estimates[direction] = (
                    0.5 * self.speed_estimates[direction] + 0.5 * measured_speed
                )
            current = next_height

        return (0 if abs(target - current) <= self.tolerance else 7), current

    def _send_until_brake_point(
        self,
        packet: bytes,
        direction: str,
        target: float,
        brake_distance: float,
        max_seconds: float,
    ) -> tuple[float | None, float]:
        brake_height = target - brake_distance if direction == "up" else target + brake_distance
        started_at = time.monotonic()
        last_height = None

        with self._lock:
            ser = self._serial
            if ser is None:
                return None, 0.0
            while (
                time.monotonic() - started_at < max_seconds
                and not self._stop_event.is_set()
                and not self._retarget_event.is_set()
            ):
                ser.write(packet)
                ser.flush()
                chunk = ser.read(256)
                if chunk:
                    self._buffer.extend(chunk)
                    for height in pop_heights(self._buffer):
                        last_height = height
                        self._publish_height(height)
                        if direction == "up" and height >= brake_height:
                            return last_height, time.monotonic() - started_at
                        if direction == "down" and height <= brake_height:
                            return last_height, time.monotonic() - started_at
                time.sleep(self.interval)
        return last_height, time.monotonic() - started_at

    def _settle_and_measure(self, direction: str, target: float) -> float | None:
        samples: list[float] = []
        brake_sent = False
        deadline = time.monotonic() + self.motion_options.settle_seconds

        with self._lock:
            ser = self._serial
            if ser is None:
                return None
            while (
                time.monotonic() < deadline
                and not self._stop_event.is_set()
                and not self._retarget_event.is_set()
            ):
                chunk = ser.read(64)
                if not chunk:
                    continue
                self._buffer.extend(chunk)
                for height in pop_heights(self._buffer):
                    samples.append(height)
                    self._publish_height(height)
                    crossed = (
                        (direction == "up" and height >= target)
                        or (direction == "down" and height <= target)
                    )
                    if crossed and not brake_sent and self.motion_options.active_brake_packets > 0:
                        brake_sent = True
                        brake_packet = DOWN if direction == "up" else UP
                        for _ in range(self.motion_options.active_brake_packets):
                            ser.write(brake_packet)
                            ser.flush()
                            time.sleep(self.interval)
        return samples[-1] if samples else None

    def _stop_callback(self, _request: Trigger.Request, response: Trigger.Response):
        self._stop_event.set()
        with self._target_lock:
            self._latest_target = None
            self._target_event.clear()
            self._retarget_event.clear()
        response.success = True
        response.message = "Lift stop requested"
        self._publish_status(response.message)
        return response

    def _reconnect_callback(self, _request: Trigger.Request, response: Trigger.Response):
        response.success = self._connect()
        response.message = "Connected" if response.success else "Connection failed; see node log"
        return response

    def destroy_node(self) -> bool:
        self._shutdown_event.set()
        self._stop_event.set()
        if self._worker.is_alive():
            self._worker.join(timeout=1.0)
        with self._lock:
            self._disconnect_locked()
        return super().destroy_node()


def main(args=None) -> None:
    rclpy.init(args=args)
    node = LiftNode()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == "__main__":
    main()
