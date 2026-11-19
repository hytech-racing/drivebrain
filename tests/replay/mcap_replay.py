from mcap.reader import NonSeekingReader
import zmq, time
import sys
from foxglove_schemas_protobuf.PointCloud_pb2 import PointCloud

# Socket ports
# drivebrain connects to tcp://$SIM_HOST:<port>, so bind on all interfaces
ENDPOINT_PREFIX = "tcp://*:"

RECV_SOCKET_PORT = 6767
SEND_SOCKET_PORT = 5940 
LIDAR_SOCKET_PORT = 1155

BLACKLIST = {
    "estimator_Outports", "Cones", "SceneUpdate", "ouster/calibration",
    # recorded outputs of drivebrain's own odometry/perception/SLAM, which would compete with the live ones
    "tf", "odometry", "filtered_cones", "slam_landmarks",
    "slam/vehicle_pose", "slam/frontend_debug", "slam/backend_debug",
    "blackfly_26191748/cones", "blackfly_26291609/cones",
}

# Initialize the sockets
ctx = zmq.Context()

recv_sock = ctx.socket(zmq.PUSH)
recv_sock.bind(f"{ENDPOINT_PREFIX}{RECV_SOCKET_PORT}")

send_sock = ctx.socket(zmq.PULL)
send_sock.bind(f"{ENDPOINT_PREFIX}{SEND_SOCKET_PORT}")

lidar_sock = ctx.socket(zmq.PUSH)
lidar_sock.bind(f"{ENDPOINT_PREFIX}{LIDAR_SOCKET_PORT}")


if (len(sys.argv) != 2): 
    sys.exit("Error: Expected usage python3 mcap_replay.py mcap/<mcap-name>.mcap")

mcap_path = sys.argv[1]

input("drivebrain up? press enter to start replay...")

with open(mcap_path, "rb") as f:
    # read front to back like Foxglove, so recordings cut off before their summary still replay
    reader = NonSeekingReader(f)
    wall_start = time.monotonic()
    mcap_start = None
    for schema, channel, message in reader.iter_messages():
        if channel.message_encoding != "protobuf":
            continue
        if channel.topic in BLACKLIST:
            continue
        if mcap_start is None:
            mcap_start = message.log_time
        target = wall_start + (message.log_time - mcap_start) / 1e9
        dt = target - time.monotonic()
        if dt > 0:
            time.sleep(dt)

        if channel.topic in ("PointCloud", "ouster/points"):
            # restamp to now so StateTracker doesn't drop the scan as stale
            pc = PointCloud()
            pc.ParseFromString(message.data)
            pc.timestamp.GetCurrentTime()
            try:
                lidar_sock.send(pc.SerializeToString(), zmq.NOBLOCK)
            except zmq.Again:
                pass
        else:
            # topic lets drivebrain tell which camera a /detections message belongs to
            recv_sock.send_multipart([schema.name.encode(), channel.topic.encode(), message.data])

recv_sock.close()
send_sock.close()
lidar_sock.close()
ctx.term()