import serial
import time
import threading
import re
import math
from collections import deque
from dataclasses import dataclass
from typing import Optional, List
import numpy as np

@dataclass
class Pose:
    x: float
    y: float
    theta: float
    vx: float = 0.0
    omega: float = 0.0

@dataclass
class Scan:
    angles: np.ndarray
    ranges: np.ndarray

@dataclass
class PoseSetpoint:
    x: float
    y: float
    theta: float
    v: float = 0.3
    omega: float = 0.0

_serial_port: Optional[serial.Serial] = None
_receiver_thread: Optional[threading.Thread] = None
_running = False

_latest_imu_data = {
    'yaw': 0.0,
    'roll': 0.0,
    'pitch': 0.0,
    'accel': [0, 0, 0],
    'gyro': [0, 0, 0],
    'timestamp': 0.0
}

_lidar_buffer = deque(maxlen=500)

_odometry = {
    'x': 0.0,
    'y': 0.0,
    'theta': 0.0,
    'vx': 0.0,
    'omega': 0.0
}

_data_lock = threading.Lock()

def init(port: str = 'COM5', baudrate: int = 115200, timeout: float = 0.1) -> None:
    global _serial_port, _receiver_thread, _running

    if _serial_port is not None:
        print("[RobotSerial] Warning: Serial port is already open, closing old connection first")
        close()

    try:
        _serial_port = serial.Serial(
            port=port,
            baudrate=baudrate,
            bytesize=serial.EIGHTBITS,
            parity=serial.PARITY_NONE,
            stopbits=serial.STOPBITS_ONE,
            timeout=timeout
        )
        print(f"[RobotSerial] Successfully opened serial port: {port} @ {baudrate} bps")
        print(f"!!!VERSION_2025_10_21_MODIFIED!!!", flush=True)

        print(f"[DEBUG] Buffer in_waiting={_serial_port.in_waiting}", flush=True)

        _running = True
        _receiver_thread = threading.Thread(target=_receive_loop, daemon=True)
        _receiver_thread.start()
        print("[RobotSerial] Receiver thread started")

        import time
        time.sleep(0.1)
        print(f"[DEBUG] Thread alive status: {_receiver_thread.is_alive()}", flush=True)

    except serial.SerialException as e:
        print(f"[RobotSerial] Failed to open serial port: {e}")
        raise


def close() -> None:
    global _serial_port, _running, _receiver_thread

    _running = False

    if _receiver_thread is not None:
        _receiver_thread.join(timeout=2.0)
        _receiver_thread = None

    if _serial_port is not None:
        _serial_port.close()
        _serial_port = None
        print("[RobotSerial] Serial port closed")


def _receive_loop() -> None:
    try:
        with open("d:\\硬件\\thread_alive.txt", "w") as f:
            f.write("THREAD ENTERED\n")
    except:
        pass

    global _serial_port, _running

    import sys
    sys.stdout.flush()
    sys.stderr.flush()

    try:
        print("="*50, flush=True)
        print("[DEBUG] === Receive loop started ===", flush=True)
        print("="*50, flush=True)
    except Exception as e:
        import traceback
        traceback.print_exc()
        return

    line_buffer = ""
    unified_buffer = bytearray()
    sync_loss_count = 0

    while _running and _serial_port is not None:
        try:
            if _serial_port.in_waiting > 0:
                data = _serial_port.read(_serial_port.in_waiting)
                unified_buffer.extend(data)

                i = 0
                packets_found = 0
                while i < len(unified_buffer):
                    if (i + 1 < len(unified_buffer) and
                        unified_buffer[i] == 0xAA and
                        unified_buffer[i+1] == 0x55):

                        if i + 5 <= len(unified_buffer):
                            msg_type = unified_buffer[i+2]
                            length = unified_buffer[i+3] | (unified_buffer[i+4] << 8)
                            packet_len = 7 + length

                            if packet_len > 256:
                                i += 1
                                continue

                            if i + packet_len <= len(unified_buffer):
                                packet = bytes(unified_buffer[i:i+packet_len])
                                packets_found += 1

                                if msg_type == 0x11 and length == 24:
                                    _parse_odometry_packet(packet)
                                    sync_loss_count = 0
                                elif msg_type == 0x12 and length == 28:
                                    _parse_imu_packet(packet)
                                    sync_loss_count = 0
                                else:
                                    pass

                                unified_buffer = unified_buffer[:i] + unified_buffer[i+packet_len:]
                                continue

                        break

                    i += 1

                if len(unified_buffer) > 512 and packets_found == 0:
                    sync_loss_count += 1
                    if sync_loss_count > 3:
                        print(f"[WARNING] Communication synchronization lost! Buffer size={len(unified_buffer)}, clearing buffer", flush=True)
                        unified_buffer.clear()
                        sync_loss_count = 0

                if len(unified_buffer) > 0:
                    try:
                        text_end = len(unified_buffer)
                        for j in range(len(unified_buffer) - 1):
                            if unified_buffer[j] == 0xAA and unified_buffer[j+1] == 0x55:
                                text_end = j
                                break

                        if text_end > 0:
                            text_data = bytes(unified_buffer[:text_end])
                            text = text_data.decode('utf-8', errors='ignore')
                            line_buffer += text
                            unified_buffer = unified_buffer[text_end:]

                            while '\n' in line_buffer:
                                line, line_buffer = line_buffer.split('\n', 1)
                                line = line.strip()
                                if line:
                                    _parse_line(line)
                    except:
                        pass

                if len(unified_buffer) > 2048:
                    unified_buffer = unified_buffer[1024:]

            else:
                time.sleep(0.01)

        except Exception as e:
            if _running:
                print(f"[RobotSerial] Receive error: {e}")
            time.sleep(0.1)


def _parse_odometry_packet(packet: bytes) -> None:
    import struct

    try:
        if len(packet) < 31:
            return

        payload = packet[5:29]
        timestamp, x, y, theta, v_linear, v_angular = struct.unpack('<I5f', payload)

        with _data_lock:
            _odometry['x'] = x
            _odometry['y'] = y
            _odometry['theta'] = theta
            _odometry['vx'] = v_linear
            _odometry['omega'] = v_angular
            _latest_imu_data['timestamp'] = timestamp

    except Exception as e:
        print(f"[RobotSerial] Odometry packet parsing error: {e}")


def _parse_imu_packet(packet: bytes) -> None:
    import struct

    try:
        if len(packet) < 35:
            return

        payload = packet[5:33]
        timestamp, yaw, roll, pitch, ax, ay, az, gx, gy, gz = struct.unpack('<I3f6h', payload)

        with _data_lock:
            _latest_imu_data['yaw'] = yaw
            _latest_imu_data['roll'] = roll
            _latest_imu_data['pitch'] = pitch
            _latest_imu_data['accel'] = [ax, ay, az]
            _latest_imu_data['gyro'] = [gx, gy, gz]
            _latest_imu_data['timestamp'] = time.time()

    except Exception as e:
        print(f"[RobotSerial] IMU packet parsing error: {e}")


def _parse_line(line: str) -> None:
    global _latest_imu_data, _lidar_buffer, _odometry, _data_lock

    if '"angles":' in line:
        try:
            angles_match = re.search(r'"angles":\[([-\d.]+),([-\d.]+),([-\d.]+)\]', line)
            accel_match = re.search(r'"accel":\[([-\d]+),([-\d]+),([-\d]+)\]', line)
            gyro_match = re.search(r'"gyro":\[([-\d]+),([-\d]+),([-\d]+)\]', line)

            if angles_match:
                with _data_lock:
                    _latest_imu_data['yaw'] = float(angles_match.group(1))
                    _latest_imu_data['roll'] = float(angles_match.group(2))
                    _latest_imu_data['pitch'] = float(angles_match.group(3))
                    _latest_imu_data['timestamp'] = time.time()

                    _odometry['theta'] = np.deg2rad(_latest_imu_data['yaw'])

            if accel_match:
                _latest_imu_data['accel'] = [int(accel_match.group(1)),
                                              int(accel_match.group(2)),
                                              int(accel_match.group(3))]

            if gyro_match:
                _latest_imu_data['gyro'] = [int(gyro_match.group(1)),
                                             int(gyro_match.group(2)),
                                             int(gyro_match.group(3))]
        except Exception as e:
            print(f"[RobotSerial] IMU data parsing failed: {line} | {e}")

    elif line.startswith('Lidar:'):
        try:
            match = re.search(r'A=([-\d.]+),\s*D=([-\d.]+)mm,\s*Q=(\d+)', line)
            if match:
                angle_deg = float(match.group(1))
                distance_mm = float(match.group(2))
                quality = int(match.group(3))

                with _data_lock:
                    _lidar_buffer.append((angle_deg, distance_mm, quality))
        except Exception as e:
            print(f"[RobotSerial] Lidar data parsing failed: {line} | {e}")

    elif line.startswith('PID:'):
        print(f"[RobotSerial-DEBUG] {line}")


def get_robot_pose(timeout: float = 1.0) -> Pose:
    start_time = time.time()

    while time.time() - start_time < timeout:
        with _data_lock:
            if _latest_imu_data['timestamp'] > 0:
                import math
                theta_normalized = math.atan2(math.sin(_odometry['theta']), math.cos(_odometry['theta']))

                pose = Pose(
                    x=_odometry['x'],
                    y=_odometry['y'],
                    theta=theta_normalized,
                    vx=_odometry['vx'],
                    omega=_odometry['omega']
                )

                if not hasattr(get_robot_pose, '_call_count'):
                    get_robot_pose._call_count = 0
                get_robot_pose._call_count += 1
                if get_robot_pose._call_count % 10 == 0:
                    print(f"[POSE] x={pose.x:.3f}m y={pose.y:.3f}m θ={math.degrees(pose.theta):.1f}° "
                          f"v={pose.vx:.2f}m/s ω={pose.omega:.2f}rad/s")

                return pose
        time.sleep(0.01)

    with _data_lock:
        print(f"[ERROR-TIMEOUT] No pose data received!", flush=True)
        print(f"  IMU timestamp: {_latest_imu_data['timestamp']}", flush=True)
        print(f"  Odometry: x={_odometry['x']:.3f}, y={_odometry['y']:.3f}, θ={_odometry['theta']:.3f}", flush=True)
        print(f"  Hint: Check if STM32 is sending odometry packets properly (MSG_TYPE_ODOMETRY=0x11)", flush=True)
    raise TimeoutError(f"Did not receive pose data within {timeout}s")


def get_lidar_scan(timeout: float = 2.0, min_points: int = 20) -> Scan:
    start_time = time.time()

    while time.time() - start_time < timeout:
        with _data_lock:
            if len(_lidar_buffer) >= min_points:
                points = list(_lidar_buffer)

                _lidar_buffer.clear()

                angles_deg = np.array([p[0] for p in points])
                distances_mm = np.array([p[1] for p in points])

                angles_rad = np.deg2rad(angles_deg)
                ranges_m = distances_mm / 1000.0

                if not hasattr(get_lidar_scan, '_call_count'):
                    get_lidar_scan._call_count = 0
                get_lidar_scan._call_count += 1
                if get_lidar_scan._call_count % 20 == 0:
                    print(f"[LIDAR] Obtained scan: {len(points)} points, "
                          f"angle range[{angles_deg.min():.1f}°, {angles_deg.max():.1f}°], "
                          f"distance range[{ranges_m.min():.2f}m, {ranges_m.max():.2f}m]")

                return Scan(angles=angles_rad, ranges=ranges_m)

        time.sleep(0.05)

    with _data_lock:
        current_points = len(_lidar_buffer)
    print(f"[ERROR-TIMEOUT] Insufficient lidar data!", flush=True)
    print(f"  Required: {min_points} points, Current: {current_points} points", flush=True)
    print(f"  Hint: Check if lidar is working properly and if serial port is receiving Lidar data", flush=True)
    raise TimeoutError(f"Did not receive sufficient lidar data within {timeout}s (current points: {current_points})")


def send_setpoint(sp: PoseSetpoint, retry: int = 3) -> bool:
    global _serial_port
    import struct

    if _serial_port is None:
        print("[RobotSerial] Error: Serial port not open")
        return False

    for attempt in range(retry):
        try:
            seq = getattr(send_setpoint, '_seq_counter', 0)
            send_setpoint._seq_counter = (seq + 1) % 65536

            payload = struct.pack('<H5f',
                seq,
                sp.x,
                sp.y,
                sp.theta,
                sp.v,
                sp.omega
            )

            header = bytes([0xAA, 0x55])
            msg_type = 0x01
            length = len(payload)
            length_bytes = struct.pack('<H', length)

            crc_data = bytes([msg_type]) + length_bytes + payload
            crc16 = _calc_crc16(crc_data)
            crc_bytes = struct.pack('<H', crc16)

            packet = header + bytes([msg_type]) + length_bytes + payload + crc_bytes

            _serial_port.write(packet)
            _serial_port.flush()

            if not hasattr(send_setpoint, '_send_count'):
                send_setpoint._send_count = 0
            send_setpoint._send_count += 1
            if send_setpoint._send_count % 10 == 0:
                print(f"[CMD-BIN] Setpoint seq={seq} | Target:({sp.x:.2f},{sp.y:.2f},{math.degrees(sp.theta):.1f}°) "
                      f"v={sp.v:.2f}m/s ω={sp.omega:.2f}rad/s | Sent {len(packet)} bytes")

            return True

        except Exception as e:
            if attempt < retry - 1:
                print(f"[RobotSerial] Sending failed (attempt {attempt+1}/{retry}): {e}, retrying...")
                time.sleep(0.05)
            else:
                print(f"[RobotSerial] Failed to send pose setpoint (retried {retry} times): {e}")
                import traceback
                traceback.print_exc()

                print(f"[RobotSerial] Attempting to reconnect serial port...")
                try:
                    close()
                    time.sleep(0.5)
                except:
                    pass

                return False

    return False


def _calc_crc16(data: bytes) -> int:
    crc = 0xFFFF
    for byte in data:
        crc ^= (byte << 8)
        for _ in range(8):
            if crc & 0x8000:
                crc = (crc << 1) ^ 0x1021
            else:
                crc <<= 1
            crc &= 0xFFFF
    return crc


def set_odometry_pose(x: float, y: float, theta: float) -> bool:
    global _serial_port
    import struct

    if _serial_port is None:
        print("[RobotSerial] Error: Serial port not open")
        return False

    try:
        payload = struct.pack('<3f', x, y, theta)

        header = bytes([0xAA, 0x55])
        msg_type = 0x02
        length = len(payload)
        length_bytes = struct.pack('<H', length)

        crc_data = bytes([msg_type]) + length_bytes + payload
        crc16 = _calc_crc16(crc_data)
        crc_bytes = struct.pack('<H', crc16)

        packet = header + bytes([msg_type]) + length_bytes + payload + crc_bytes

        _serial_port.write(packet)
        _serial_port.flush()

        print(f"[ODOM-SET] Set odometry initial position: x={x:.3f}m y={y:.3f}m θ={math.degrees(theta):.1f}° | Sent {len(packet)} bytes")
        return True

    except Exception as e:
        print(f"[RobotSerial] Failed to set odometry initial position: {e}")
        import traceback
        traceback.print_exc()
        return False


def get_latest_imu_data() -> dict:
    with _data_lock:
        return _latest_imu_data.copy()


def get_lidar_buffer_size() -> int:
    with _data_lock:
        return len(_lidar_buffer)


def print_status() -> None:
    imu = get_latest_imu_data()
    lidar_size = get_lidar_buffer_size()

    print("\n========== Robot Serial Status ==========")
    print(f"Serial port status: {'Connected' if _serial_port and _serial_port.is_open else 'Disconnected'}")
    print(f"Receiver thread: {'Running' if _running else 'Stopped'}")
    print(f"\nIMU data:")
    print(f"  Yaw:   {imu['yaw']:.2f}°")
    print(f"  Roll:  {imu['roll']:.2f}°")
    print(f"  Pitch: {imu['pitch']:.2f}°")
    print(f"  Accel: {imu['accel']}")
    print(f"  Gyro:  {imu['gyro']}")
    print(f"  Update time: {time.time() - imu['timestamp']:.2f}s ago")
    print(f"\nLidar buffer: {lidar_size} points")
    print(f"\nOdometry estimate:")
    print(f"  Position: ({_odometry['x']:.3f}, {_odometry['y']:.3f}) m")
    print(f"  Heading: {np.rad2deg(_odometry['theta']):.2f}°")
    print("=========================================\n")


if __name__ == '__main__':
    import sys

    port = sys.argv[1] if len(sys.argv) > 1 else 'COM5'

    print(f"Connecting to {port}...")
    init(port=port, baudrate=115200)

    try:
        print("Waiting for data...(Press Ctrl+C to exit)\n")

        while True:
            time.sleep(3)
            print_status()

            try:
                pose = get_robot_pose(timeout=0.5)
                print(f"✓ Successfully obtained pose: x={pose.x:.3f}, y={pose.y:.3f}, theta={np.rad2deg(pose.theta):.1f}°")
            except TimeoutError as e:
                print(f"✗ Failed to obtain pose: {e}")

            try:
                scan = get_lidar_scan(timeout=1.0, min_points=10)
                print(f"✓ Successfully obtained lidar scan: {len(scan.ranges)} points")
            except TimeoutError as e:
                print(f"✗ Failed to obtain lidar scan: {e}")

    except KeyboardInterrupt:
        print("\nUser interrupted")
    finally:
        close()
        print("Program ended")