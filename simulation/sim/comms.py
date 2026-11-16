import math
import time
import types

import numpy as np
import zmq

import os

from foxglove_schemas_protobuf.FrameTransform_pb2 import FrameTransform
from foxglove_schemas_protobuf.PackedElementField_pb2 import PackedElementField
from foxglove_schemas_protobuf.PointCloud_pb2 import PointCloud

from .vehicle import VehicleInput

SIM_BIND_HOST: str = os.environ.get("SIM_BIND_HOST", "127.0.0.1")

def _endpoint(port: int) -> str:
    return f"tcp://{SIM_BIND_HOST}:{port}"

SEND_SOCKET_PORT: int = 6767
RECV_SOCKET_PORT: int = 5940
LIDAR_SOCKET_PORT: int = 1155

INPUT_TIMEOUT: float = 1.0 # secs

POINT_DTYPE = np.dtype([('x', '<f4'), ('y', '<f4'), ('z', '<f4')])
POINT_STRIDE: int = POINT_DTYPE.itemsize  # 12

def _all_finite(msg) -> bool:
    """A single NaN/inf input poisons the integrator and RK4 state for the rest of the run"""
    return all(math.isfinite(v) for _, v in msg.ListFields())

class SimComms:
    def __init__(self, proto: types.ModuleType, can: types.ModuleType) -> None:
        self._proto = proto
        self._can = can
        self._ctx: zmq.Context = zmq.Context()

        # NOTE: socket options must be set BEFORE bind() to apply to every pipe a
        # peer opens; setting them after leaves an early-connecting consumer on the
        # 1000-deep default queue, which is exactly how a backlog builds.
        self._cmd: zmq.Socket = self._ctx.socket(zmq.PULL)
        self._cmd.setsockopt(zmq.RCVHWM, 1)
        self._cmd.bind(_endpoint(RECV_SOCKET_PORT))

        # multipart channel. carries all low-rate ground-truth data (pose, cones, transforms).
        # multipart rules out CONFLATE, so keep the queue shallow to bound transform staleness.
        self._state: zmq.Socket = self._ctx.socket(zmq.PUSH)
        self._state.setsockopt(zmq.SNDHWM, 4)
        self._state.setsockopt(zmq.LINGER, 0)
        self._state.bind(_endpoint(SEND_SOCKET_PORT))

        # lidar is single-part and only-latest-matters -> CONFLATE keeps just the newest
        # scan queued, so a slow Foxglove/bridge can never accumulate stale clouds.
        self._lidar: zmq.Socket = self._ctx.socket(zmq.PUSH)
        self._lidar.setsockopt(zmq.CONFLATE, 1)
        self._lidar.setsockopt(zmq.LINGER, 0)
        self._lidar.bind(_endpoint(LIDAR_SOCKET_PORT))

        self._latest_input: VehicleInput = VehicleInput()
        self._last_cmd_time: float = time.monotonic()
        self._start_time: float = time.monotonic()

    def drain_commands(self) -> None:
        while True:
            try:
                parts: list[bytes] = self._cmd.recv_multipart(zmq.NOBLOCK)
            except zmq.Again:
                break
            if len(parts) < 2:
                continue
            self._handle_command(parts[0].decode(), parts[1])

    def _handle_command(self, type_name: str, body: bytes) -> None:
        """Unpack a drivebrain CAN command into the latest vehicle input."""
        if type_name == "hytech.drivebrain_speed_set_input":
            msg = self._can.hytech_pb2.drivebrain_speed_set_input()
            msg.ParseFromString(body)
            if not _all_finite(msg):
                return
            self._latest_input.speed_rpm_fl = msg.drivebrain_set_rpm_fl
            self._latest_input.speed_rpm_fr = msg.drivebrain_set_rpm_fr
            self._latest_input.speed_rpm_rl = msg.drivebrain_set_rpm_rl
            self._latest_input.speed_rpm_rr = msg.drivebrain_set_rpm_rr
        elif type_name == "hytech.drivebrain_torque_lim_input":
            msg = self._can.hytech_pb2.drivebrain_torque_lim_input()
            msg.ParseFromString(body)
            if not _all_finite(msg):
                return
            self._latest_input.torque_lim_fl = msg.drivebrain_torque_fl
            self._latest_input.torque_lim_fr = msg.drivebrain_torque_fr
            self._latest_input.torque_lim_rl = msg.drivebrain_torque_rl
            self._latest_input.torque_lim_rr = msg.drivebrain_torque_rr
        elif type_name == "hytech.drivebrain_steering_input":
            msg = self._can.hytech_pb2.drivebrain_steering_input()
            msg.ParseFromString(body)
            if not _all_finite(msg):
                return
            self._latest_input.wheel_steer_rad = math.radians(msg.drivebrain_steering)
        else:
            return

        self._last_cmd_time = time.monotonic()

    def timed_out(self) -> bool:
        return time.monotonic() - self._last_cmd_time > INPUT_TIMEOUT

    def current_input(self) -> VehicleInput:
        """Return the latest input, zeroing setpoints and torque limits if the controller timed out."""
        if self.timed_out():
            return VehicleInput(wheel_steer_rad=self._latest_input.wheel_steer_rad)
        return self._latest_input

    def _send_typed(self, msg) -> None:
        """Multipart (type name, body) frame"""
        try:
            self._state.send_multipart(
                [msg.DESCRIPTOR.full_name.encode(), msg.SerializeToString()], zmq.NOBLOCK
            )
        except zmq.Again:
            pass

    def send_pose(self, x: float, y: float, psi: float) -> None:
        """Ground-truth vehicle pose as hytech_msgs.pose, map frame."""
        msg = self._proto.base_msgs_pb2.pose()
        msg.position.x = x
        msg.position.y = y
        msg.orientation.z = math.sin(psi * 0.5)
        msg.orientation.w = math.cos(psi * 0.5)
        self._send_typed(msg)

    def send_pointcloud(self, pts: np.ndarray) -> None:
        n: int = len(pts)
        buf = np.empty(n, dtype=POINT_DTYPE)
        buf['x'] = pts[:, 0]
        buf['y'] = pts[:, 1]
        buf['z'] = pts[:, 2]

        PEF = PackedElementField
        msg = PointCloud(
            frame_id="lidar",
            point_stride=POINT_STRIDE,
            fields=[PEF(name="x", offset=0, type=PEF.FLOAT32),
                    PEF(name="y", offset=4, type=PEF.FLOAT32),
                    PEF(name="z", offset=8, type=PEF.FLOAT32),
            ],
            data=buf.tobytes(),
        )
        # capture time — consumers derive scan age from this, and slam needs it to pair
        # a scan with odometry. wall clock, to match how drivebrain stamps its logs.
        msg.timestamp.GetCurrentTime()
        try:
            self._lidar.send(msg.SerializeToString(), zmq.NOBLOCK)
        except zmq.Again:
            pass

    def send_cones(self, cone_xy: np.ndarray, cone_type: np.ndarray) -> None:
        """Ground-truth track map. cone_type maps 1:1 to dv_msgs Cones.ConeColor."""
        msg = self._proto.dv_msgs_pb2.Cones(timestamp_us=int(time.time() * 1e6))
        for (x, y), c in zip(cone_xy, cone_type):
            cone = msg.cones.add()
            cone.position.x = float(x)
            cone.position.y = float(y)
            cone.color = int(c)
        self._send_typed(msg)

    def send_oms(self, vx: float, vy: float, yaw_rate: float) -> None:
        """Ground speed sensor measurement as hytech_msgs.OMSData. vx, vy are the velocity at the
        sensor mount in body frame (FLU, m/s), yaw_rate in rad/s."""
        msg = self._proto.hytech_msgs_pb2.OMSData()
        # sensor-local clock like the real unit; drivebrain maps it onto host time itself.
        # relative to start since the field is a float32
        msg.sample_time_s = time.monotonic() - self._start_time
        # the real sensor is mounted facing backwards and drivebrain's Odometry negates
        # vel_x/vel_y to undo it, so report in the sensor's own (rear-facing) frame
        msg.motion.vel_x_kmh = -vx * 3.6
        msg.motion.vel_y_kmh = -vy * 3.6
        msg.motion.vel_a_kmh = math.hypot(vx, vy) * 3.6
        msg.motion.ang_s_deg = math.degrees(math.atan2(vy, vx))
        msg.rate_hor_deg_s.z = math.degrees(yaw_rate)
        msg.rate_deg_s.z = msg.rate_hor_deg_s.z
        msg.status.standstill = vx == 0.0 and vy == 0.0
        msg.status.status_ok = True
        msg.status.sensor_connected = True
        msg.status.illu_ok = True
        self._send_typed(msg)

    def send_transform(self, x: float, y: float, z: float, psi: float) -> None:
        """map -> lidar transform"""
        msg = FrameTransform(parent_frame_id="map", child_frame_id="lidar")
        msg.timestamp.GetCurrentTime()
        msg.translation.x = x
        msg.translation.y = y
        msg.translation.z = z
        msg.rotation.z = math.sin(psi * 0.5)
        msg.rotation.w = math.cos(psi * 0.5)
        self._send_typed(msg)

    def close(self) -> None:
        self._cmd.close()
        self._state.close(linger=0)
        self._lidar.close()
        self._ctx.term()
