# Ethernet camera synchronization

Drivebrain arms the GigE cameras with `FrameStart` / `Action0` and sends
acknowledged GigE Vision scheduled action commands at 15 Hz. Every camera receives
the same future PTP timestamp. Commands are queued 200 ms ahead; late host wakeups
skip slots for the entire group instead of sending past-due triggers.

Requirements:

- Cameras supporting PTP, scheduled actions, `ActionQueueEmpty`, and at least
  eight action queue entries (Blackfly S GigE firmware exposes these features).
- The FLIR `GevIEEE1588*` mode, latched status, offset, and parent-ID nodes, and a
  1 GHz timestamp clock. Unsupported models fail camera setup explicitly.
- An external PTP master on the camera Ethernet network. Configure the Jetson's
  PTP service and NIC separately; Drivebrain does not start `ptp4l` or `phc2sys`.
  Both cameras run in `SlaveOnly` mode. Before scheduling, they must report the
  same parent and offsets within 100 us for three consecutive samples.
- Exposure/readout must fit the 66.7 ms frame period. Use the same exposure
  settings for both cameras when exposure duration matters to stereo matching.

Failure to initialize any discovered camera or obtain PTP lock disables the
camera group while the rest of Drivebrain continues. During capture, PTP is
checked once per second; losing lock or receiving a failed/missing action ACK
stops scheduling and clears queued actions. A failure after one camera accepts a
trigger can still leave an unmatched frame. Restart Drivebrain after fixing the
cause; there is no automatic fallback to free-running acquisition.

Image and annotation protobuf timestamps now come from the camera buffer's
capture timestamp, in the PTP master's time domain. MCAP envelope log/publish
times still use the host logging time. Ensure the PTP master's epoch/time scale
is configured appropriately for comparison with other sensors (PTP TAI and host
UTC can differ). Image payload compression is unchanged.

## On-car verification

1. Confirm that the PTP master is running and start Drivebrain. Check for
   `Armed 2 cameras for PTP scheduled capture at 15 Hz`, and no lock-loss,
   scheduler-overrun, or action-rejection errors during the run.
2. Record at least a minute, including CPU load from detection and logging.
3. Run `python3 scripts/check_camera_sync.py recordings/SESSION.mcap`. It reads
   the embedded capture timestamps on the two `/perception` topics, matches
   frames once each within 200 us, and reports unmatched frames. A nonzero exit
   status means missing topics, no pairs, or unmatched frames; investigate packet
   loss and startup/shutdown boundaries as well as timing.
4. Verify exposure alignment with a flashing LED or moving target visible to
   both cameras. Timestamp agreement alone does not measure actual sensor skew.

The wire-format regression test can run without cameras or project dependencies:

```sh
c++ -std=c++20 -Idrivebrain_comms/include tests/sil/test_camera_sync.cpp -o /tmp/test_camera_sync
/tmp/test_camera_sync
python3 tests/sil/test_check_camera_sync.py
```

The Python recording tests require `mcap`, `mcap-protobuf-support`, and
`foxglove-schemas-protobuf` from `requirements.txt`.

Protocol references: [FLIR scheduled action commands](https://softwareservices.flir.com/bfs-pge-244s8/latest/Model/public/ActionControl.html),
[FLIR PTP nodes](https://softwareservices.flir.com/BFS-PGE-13Y3/latest/Model/public/GigEVision.html),
and [Aravis camera buffer timestamps](https://aravisproject.github.io/docs/aravis-0.8/ArvBuffer.html).
