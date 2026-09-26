# System metrics in MCAP

`drivebrain_system_metrics` is a JSON channel sampled by a background thread
approximately once per second. It uses the existing MCAP writer queue; no procfs
or sysfs reads, JSON construction, or metrics callbacks run in the control loop.
The first sample follows a one-second CPU-counter baseline. The worker is joined
before the application destroys the logger. It does not broadcast to Foxglove.

Fields:

- `sample_interval_s`: actual monotonic interval between samples.
- `cpu_usage_percent`: total CPU utilization, 0–100% across all cores.
- `cpu_usage_percent_per_core`: `cpu0`, `cpu1`, etc., each 0–100%.
- `process_cpu_usage_percent`: DriveBrain CPU usage; 100% means one fully used
  core, so a multithreaded process can exceed 100%.
- `process_rss_bytes`: current resident process memory (kernel approximate RSS).
- `memory_available_bytes`, `memory_total_bytes`: system memory in bytes.
- `thermal_zone0_temperature_c`, `thermal_zone0_type`: optional thermal-zone
  reading and its kernel label. On the Pi this is normally the CPU thermal zone;
  retain the label because zone 0 is not necessarily CPU temperature elsewhere.

CPU utilization uses `/proc/stat` deltas, treating idle and iowait as non-busy
and excluding guest fields already counted in user/nice. Process CPU uses
`/proc/self/stat` user + system ticks divided by the actual sample interval.
Memory comes from `/proc/meminfo` and `/proc/self/status`; temperature comes from
`/sys/class/thermal/thermal_zone0`. Missing/invalid readings and unusable CPU
deltas are null, never fabricated zeroes. Missing CPUs are omitted from the map.
Non-Linux simulation environments therefore produce unavailable Linux metrics.

Plot these fields alongside `drivebrain_loop_overrun.overrun_us` in a recorded
MCAP. One-second averages can reveal sustained load but cannot explain every
millisecond-scale deadline miss. No scheduling, estimator, or control behavior
is changed. The existing `McapInfo` publication rate is unchanged.

After configuring a native Linux build, run:

```
cmake --build build-native --target system_metrics_test
ctest --test-dir build-native -R '^system_metrics$' --output-on-failure
```

The test uses temporary procfs/sysfs fixtures for deltas, units, missing data,
malformed input, counter reset, and process names containing parentheses. It
also checks the background publication interval and interruptible shutdown.
