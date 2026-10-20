# Launch controller ground and jack tests

The launch controller now includes OMS ground-speed communications from
`warren/gss-comms` (`6c8af34`) and Kraken column-position control plus joystick
teleop from `krish/teleop-fix` (`893a659`). Controller indices are:

| Index | Controller |
| --- | --- |
| 0 | LoadCellTorqueController |
| 1 | LaunchController |
| 2 | TeleopController |

## Ground run

1. Configure the OMS connection at `oms/host` (default `192.168.3.50`) and
   `oms/port` (51000), with `oms/enabled=true`. These connection settings and
   the OMS unit/mounting settings below are loaded at startup.
2. Leave `launchcontroller/testing_enabled=false` and `brake_requested=false`.
   Use `use_ground_speed=true` and `heading_control_enabled=true` (the defaults).
3. Configure acceleration, distances and heading gains, then select
   `drivebraincontrollerinterface/controller_index=1`. Selection commands the
   steering column to center. The normal VCR Drivebrain/mode-4 drivetrain path
   still needs to be accepting commands and ready to drive.
4. With all wheels and ground speed stopped, toggle
   `launchcontroller/testing_enabled` false → true. This captures the current
   OMS relative heading as the straight-ahead reference for this run.
5. Acceleration continues to `accel_distance_m` of **OMS ground travel**;
   braking then begins automatically. Set `brake_requested=true` or disable
   testing to brake earlier. Heading correction continues through braking and
   centers the column when the run finishes.
6. After stopping, set both booleans false, then toggle testing true for a new
   run. Selecting another controller discards a pending start.

All four desired RPMs and torque limits remain equal. Heading correction is
exclusively a Kraken steering-column command; there is no torque vectoring in
LaunchController. Controller steering commands take priority over the Kraken
angle/sine tests and do not require `kraken_comms/test_mode=true`.
`kraken_comms/enable_output` still controls whether the motor is enabled.

## Profile and heading parameters

All parameters below are under `launchcontroller/`. Numeric profile settings,
mode switches and heading gains are captured when a run starts; changing them
in Foxglove applies to the next run. The testing/braking requests apply immediately.

| Parameter | Default | Meaning |
| --- | --- | --- |
| `acceleration_mps2` | 0.5 | Linear speed-target slope, m/s² |
| `accel_distance_m` | 5.0 | Ground travel before braking |
| `braking_distance_m` | 5.0 | Travel from braking entry to zero target |
| `max_speed_mps` | 2.0 | Hold this speed cap until acceleration distance |
| `max_torque_nm` | 2.0 | Per-motor speed-loop torque limit during acceleration |
| `max_regen_torque_nm` | 1.0 | Per-motor speed-loop torque limit during braking/hold |
| `use_ground_speed` | true | Use OMS speed/odometer for distance and braking calculations |
| `heading_control_enabled` | true | Correct launch heading with the Kraken |
| `heading_kp` | 2.0 | Column degrees per degree of heading error |
| `heading_kd` | 0.5 | Column degrees per degree/s yaw rate |
| `max_steering_column_angle_deg` | 15.0 | Maximum heading correction in column degrees |
| `steering_sign` | 1.0 | Use -1.0 to reverse the column correction convention |

The OMS stream exposes sideslip and yaw rate, **not absolute compass heading**.
The estimator integrates `rate_hor_deg_s.z` using sensor sample timestamps at
1 kHz. The controller captures a heading reference at launch, then commands:

```
error = wrap_to_180(current_heading - launch_heading)
column_target = clamp(-steering_sign * (heading_kp * error + heading_kd * yaw_rate),
                      -max_steering_column_angle_deg, max_steering_column_angle_deg)
```

Positive heading/yaw is assumed to turn left; positive column target must steer
left with `steering_sign=1`. This holds heading and damps yaw; it is not a lateral
position controller. Gyro bias causes relative-heading drift over time.

The merged Kraken implementation runs a trapezoidal position reference and
column-angle PID on Drivebrain, then sends VoltageOut to the TalonFX. The branch's
encoder initialization, steering-feedback timeout, motor-reset handling,
current limits and PID tuning are retained. This change does not add the later
mission-level failsafe work, including OMS dropout/status supervision.

## OMS units, mounting and telemetry

The updated shared schema names the velocity channels in km/h. The decoder
maps TCP words 16/17 to sensor-frame X/Y velocity, following the GSS branch's
channel map; word 15 is the horizontal yaw rate. Under `oms/`:

| Parameter | Default | Meaning |
| --- | --- | --- |
| `raw_velocity_to_mps` | 0.27777778 | Assumes raw TCP velocities in km/h; use 1.0 for m/s |
| `mounting_yaw_deg` | 0.0 | Rotate sensor X/Y velocities into vehicle forward/left axes |
| `yaw_rate_sign` | 1.0 | Use -1.0 if sensor yaw polarity is reversed |
| `yaw_rate_bias_deg_s` | 0.0 | Stationary gyro bias, subtracted before yaw sign/integration |

**Physical TCP scaling has not been checked on this car.** The km/h default
matches the updated `OMSMotion` schema and
`HT_CAN/imported_dbcs/SensoricSolutionsOMSRace.dbc`. Compare `oms/state.speed_mps * 3.6` with
the OMS web display in km/h and select the conversion accordingly. The stream's
channel ordering must match this sensor configuration. The merged recording and
matching scripts in `scripts/oms/` can compare raw channels with the web values.

Every TCP sample updates the vehicle state. Relative heading and distance use
the sensor's clock, so packet batching and the 250 Hz control loop do not change
integration. Duplicate sample times are ignored; sample-clock resets and gaps
over 100 ms establish a new integration baseline without inventing motion.

Decoded `hytech_msgs.OMSData` is recorded at full rate and streamed to Foxglove
at 100 Hz. Full raw 144-byte frames are preserved as protobuf BytesValue messages
in the `oms/raw_frame` MCAP topic, because the newer shared schema replaced its
old `raw_words` field with named motion fields. The `oms/state` topic (protobuf Struct) records/streams derived
`vx_mps`, `vy_mps`, `speed_mps`, `sideslip_deg`, `heading_deg`, `yaw_rate_deg_s`,
and `distance_m` at 100 Hz. `hytech_msgs.KrakenInfo` shows the commanded and
measured column angles and motor telemetry.

The velocity target follows `v = acceleration_mps2 * elapsed_time`, starting at
zero and capped at max speed, following the time ramp in firmware's
[SimpleLaunchController](https://github.com/hytech-racing/vehicle-firmware/blob/main/VCR/lib/systems/include/controllers/SimpleLaunchController.h).
Braking captures **OMS ground speed** at entry and calculates
`deceleration = v_entry² / (2 * braking_distance_m)`, then ramps the speed target
down linearly. It also commands zero at the measured braking distance. Actual
stopping distance depends on speed tracking and available motor braking torque;
this does not command hydraulic brakes.

## Jack mode and existing wheel feedback

For the original jack test, set both `use_ground_speed=false` and
`heading_control_enabled=false` before starting. The steering column then stays
centered and distance reverts to integrated mean absolute wheel travel. No OMS
measurement is needed in this mode.

Measured motor RPMs arrive in `hytech.inv1_dynamics` through `hytech.inv4_dynamics`,
field `actual_speed_rpm`, mapped to FL, FR, RL, RR. These are recorded/streamed
alongside `hytech.drivebrain_speed_set_input` and
`hytech.drivebrain_torque_lim_input`. Plot all four measured and requested RPMs.
The conversion uses the existing 11.86 gearbox ratio and 0.4064 m wheel diameter.
The prior requirement for fresh RPMs from every inverter remains in place.

## Tests

Run the dependency-free profile, optical odometry and heading tests:

```sh
cmake -S tests/launch -B build-launch-profile
cmake --build build-launch-profile
ctest --test-dir build-launch-profile --output-on-failure
```

For a native full build with normal dependencies configured, enable
`-DBUILD_LAUNCH_TESTS=ON` and run CTest. This additionally tests protobuf-to-state
mapping and actual Foxglove parameter updates through launch, heading correction,
ground-distance braking, and jack mode. The integration test binds port 5555;
stop other local Drivebrain instances first. Hardware speed scaling, mounting,
steering polarity and tracking still require physical verification.
