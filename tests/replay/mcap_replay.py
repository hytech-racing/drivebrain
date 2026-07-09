from mcap.reader import make_reader
import zmq, time
import sys

BLACKLIST = {"estimator_Outports"}

ctx = zmq.Context()
sock = ctx.socket(zmq.PUSH)
sock.connect("ipc:///tmp/drivebrain_sim_6767")

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
        sock.send_multipart([schema.name.encode(), message.data])

sock.close()
ctx.term()