from mcap.reader import make_reader
import zmq, time
import sys

# Socket ports
ENDPOINT_PREFIX = "ipc:///tmp/drivebrain_sim_"

RECV_SOCKET_PORT = 6767
SEND_SOCKET_PORT = 5940 
LIDAR_SOCKET_PORT = 1155

BLACKLIST = {"estimator_Outports"}

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
    reader = make_reader(f)
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

        if channel.topic == "PointCloud":
            try:
                lidar_sock.send(message.data, zmq.NOBLOCK)
            except zmq.Again:
                pass
        else:
            recv_sock.send_multipart([schema.name.encode(), message.data])

recv_sock.close()
send_sock.close()
lidar_sock.close()
ctx.term()