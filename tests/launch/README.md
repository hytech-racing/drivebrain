# Launch controller ground and jack tests

The launch controller now includes OMS ground-speed communications from
`warren/gss-comms` (`6c8af34`) and Kraken column-position control plus joystick
teleop from `krish/teleop-fix` (`893a659`). Controller indices are:

| Index | Controller |
| --- | --- |
| 0 | LoadCellTorqueController |
| 1 | TeleopController (original Xbox drive and steering) |
| 2 | LaunchController (new incremental tests) |

## Incremental testing in Foxglove

The original teleop branch's controller indices and default are preserved:
`drivebraincontrollerinterface/controller_index` defaults to **0**, and that
selection is applied at startup. Select **2** for the launch test stages below.
Index 1 retains the original Xbox drive/steering behavior and teleop settings.
The launch-controller test stage is a separate selector; it does not renumber
controllers. `launchcontroller/test_stage` is an integer that defaults to **0**:

| Stage | Test | Heading input | Travel/braking speed input |
| --- | --- | --- | --- |
| 0 | Disabled | None | None; drive commands zero, Kraken neutral |
| 1 | Original Xbox steering only | Left stick; original cubic steering response | None; all drive targets and torque limits always zero |
| 2 | Linear ramp on jacks (original `cbb7d78` bench test) | Steering held straight | Four measured wheel speeds; OMS ignored |
| 3 | Xbox fake heading on jacks | Xbox left stick horizontal axis | Four measured wheel speeds; OMS ignored |
| 4 | Remote ground-run foundation (physical launch blocked) | OMS integrated relative heading/yaw rate | OMS ground speed and travel |

1. Keep `launchcontroller/testing_enabled=false` and `brake_requested=false`.
   Select controller index 2, then stage 1, 2 or 3 for a test on jacks. Set
   acceleration, distances and speed limits before starting. The normal VCR Drivebrain/mode-4 drivetrain path
   must be accepting commands and ready to drive for stages 2–4, with fresh
   measured speeds from all four inverters and every wheel stopped to start.
   Stage 1 only steers; it needs no RTD, wheel-speed or OMS data. Kraken still
   requires its normal fresh steering-position feedback.
2. For stages 1 and 3, connect the same Xbox controller used for teleop. The
   existing reader runs even while LaunchController is selected; it shares the device
   rather than opening a second reader. Defaults are `/dev/input/js0`, axis 0
   for the left stick horizontal axis, with a 0.05 deadzone. These settings come
   from `teleopcontroller/joystick_device_path`, `steering_axis`,
   `invert_steering_axis` and `axis_deadzone`, loaded at startup. In stage 1,
   the same original teleop cubic response and live
   `teleopcontroller/max_steering_angle_deg` (default 70°) command steering:
   left steers left, right steers right. Triggers cannot command drive motors.
   In stage 3, the same stick instead simulates heading for opposite correction.
   Other axes and triggers do not command the launch drivetrain or steering.
3. For stages 1–3, toggle `launchcontroller/testing_enabled` false → true to start the selected test.
   Stage 2 keeps the steering straight. In stage 3, left-stick deflection fakes
   a leftward heading error and commands a rightward Kraken correction; right
   commands left, and centered stick commands straight. Stick position sets
   heading error directly, rather than integrating a yaw rate. Full deflection
   represents `fake_heading_max_deg` (default 10°). With the default heading
   gain, 20% stick deflection produces about 4° of opposite column correction,
   capped at 15°. No OMS messages are needed in any bench stage; incoming
   OMS speed, travel and heading are ignored completely. Stages 1 and 3 need a
   connected joystick; losing the device cancels the run without auto-restart.
4. Braking begins after `accel_distance_m` of integrated mean absolute wheel
   travel on jacks in stages 2/3. Set `brake_requested=true` for an earlier gradual
   stop using `braking_distance_m`. Stage 3 continues heading correction during
   braking. Stage 1 runs no drive profile and ignores braking requests.
5. **Setting `testing_enabled=false` or `test_stage=0` immediately clears all
   four speed targets and torque limits and forces the Kraken neutral**, even
   if an angle/sine override is enabled. This removes commanded drive; spinning
   wheels may coast. Cleared commands are transmitted even if vehicle-state
   feedback has expired. Use `brake_requested` for the gradual braking profile.
   Kraken is also neutral while waiting to start, after a fault or after
   completion; it does not drive back to center while disabled.
6. Changing stage cancels the current run. Select the new stage first, stop the
   wheels, clear `brake_requested`, and toggle testing false → true again.
   A stage change while testing is true never starts the next stage. A saved
   true testing value also cannot start at boot. Out-of-range stages disable.

All four desired RPMs and torque limits remain equal. Heading correction is
exclusively a Kraken steering-column command; there is no torque vectoring in
LaunchController. Controller steering commands take priority over the Kraken
angle/sine tests and do not require `kraken_comms/test_mode=true`.
`kraken_comms/enable_output` still controls whether the motor is enabled.
The launch enable/stage parameters govern LaunchController; selecting the
separate TeleopController at index 1 retains its manual control behavior.

## Ground run (stage 4)

Stage 4 now uses a remote mission state machine. **Physical launch is unavailable**:
the app has no RDC/EBS permission source. There is no Foxglove parameter to bypass
this restriction. The in-process feedback seam is exercised only by simulated
counterparts in tests. No firmware, CAN IDs, layouts or dependencies are changed;
Drivebrain sends no `driverless_system_status` / RDC-owned solenoid commands.
The existing DBC supplied at startup still determines receive decoding on both
configured CAN interfaces. Bus routing must be confirmed on the car.

With fresh safe RSS data, selecting stage 4 and toggling `testing_enabled`
false → true arms the test on the next controller tick. RSS `button_k3_pressed`
is observed as GO; a held GO or an edge received before arming is consumed.
A subsequent fresh GO edge creates one start request and waits for permission.
Holding GO cannot repeat the request. K2 is telemetry only. There is no RSS
operating-mode command added by this change.

Only a simulated permission grant starts the unchanged OMS distance/speed,
acceleration/braking and Kraken heading profile. Wheels and OMS speed must be
stopped at start; existing RTD, wheel feedback and profile checks still apply.
Configure `oms/host`, `oms/port`, units and mounting as before. The reference
heading is captured when permission starts the profile. Permission loss or a
profile fault enters Emergency. On completed braking, every drive command is
cleared, Kraken is neutral, and a Finished request remains pending until the
simulated counterpart acknowledges it. Stopped motion or reported EBS flags do
not imply this acknowledgment.

While armed, awaiting permission, running, or awaiting Finished acknowledgment,
either RSS stop channel becoming false, pre-alarm, incorrect mode, or RSS older
than **500 ms** latches Emergency. The timeout is a provisional Drivebrain-local
supervision limit, not a protocol change. GO and unsafe-event counters preserve
transitions between control ticks; recovered RSS outages are retained too.
`abort_requested=true`, disabling testing, changing stage or resetting/swapping
an active controller also latches Emergency. These actions clear all four RPM
and torque-limit commands and neutralize Kraken. Emergency is an **internal
request only**: this does not command hydraulic braking. RSS STOP's independent
physical effect remains unverified.

RSS recovery, a fresh GO, a testing-enable toggle and EBS emergency acknowledgment
cannot clear Emergency. To explicitly reset, keep `testing_enabled=false`, clear
`abort_requested`, provide fresh safe RSS, then toggle `reset_emergency` false →
true while LaunchController is selected. Reset is processed at a controller tick.
After reset, rearm and supply a new GO edge. The same explicit disabled reset
prepares another mission after Finished. Stages 0–3 and manual teleop retain
their existing controls and do not depend on RSS/EBS.

`remote_safety/state` (protobuf Struct, Foxglove and MCAP at 10 Hz, including
manual teleop) exposes receipt flags, steady-clock receive times, message ages,
RSS freshness, GO/K2, both stop channels, mode, pre-alarm, link quality, event
counters, both EBS air pressures and every reported EBS state flag. Pressure
values retain the pinned schema/DBC units. Original decoded RSS/EBS messages
continue to be logged and streamed by the existing CAN receiver.
`launchcontroller/remote_mission` exposes internal start/Emergency/Finished
requests, start count, emergency acknowledgment and counterpart availability
at 10 Hz while LaunchController is selected. State numbers are 0 Disarmed,
1 Armed, 2 Awaiting permission, 3 Running, 4 Awaiting Finished acknowledgment,
5 Finished, 6 Emergency. EBS reports and the unconfirmed `0x22A` layout never
authorize launch or acknowledge these requests.

Open integration questions for tonight:

- Where is FDC/EBS actuator firmware, and does RSS STOP already engage it independently?
- Which bus carries RSS/EBS/RDC traffic to Drivebrain?
- Which `0x22A` layout is deployed?
- What existing interface lets Drivebrain request emergency braking and normal mission completion?

Physical EBS integration, traction control and the larger safety-check list
remain unfinished. Sensor calibration, steering polarity and tracking still
require physical verification.

## Profile and heading parameters

All parameters below are under `launchcontroller/`. Numeric profile settings
and heading gains are captured when a run starts;
changing them in Foxglove applies to the next run. Stage changes cancel the run;
testing, braking and abort requests apply immediately; emergency reset is processed on the next controller tick. The stage parameter replaces the
old `use_ground_speed` and `heading_control_enabled` switches.

| Parameter | Default | Meaning |
| --- | --- | --- |
| `testing_enabled` | false | Enable edge starts stages 1–3; arms stage 4 |
| `brake_requested` | false | Begin the existing gradual drivetrain braking profile |
| `abort_requested` | false | Stage 4 operator abort; latches Emergency and clears commands |
| `reset_emergency` | false | Rising edge resets stage 4 only while disabled with fresh safe RSS and abort cleared |
| `acceleration_mps2` | 0.5 | Linear speed-target slope, m/s² |
| `accel_distance_m` | 5.0 | Wheel travel in stages 2/3, ground travel in stage 4, before braking |
| `braking_distance_m` | 5.0 | Travel from braking entry to zero target |
| `max_speed_mps` | 2.0 | Hold this speed cap until acceleration distance |
| `max_torque_nm` | 2.0 | Per-motor speed-loop torque limit during acceleration |
| `max_regen_torque_nm` | 1.0 | Per-motor speed-loop torque limit during braking/hold |
| `test_stage` | 0 | 0 disabled, 1 Xbox steering only, 2 bench ramp, 3 Xbox heading bench, 4 OMS ground |
| `fake_heading_max_deg` | 10.0 | Stage 3 full-stick heading error; positive, at most 180° |
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
Braking captures **measured wheel speed in stages 2/3 or OMS ground speed in stage 4** at entry and calculates
`deceleration = v_entry² / (2 * braking_distance_m)`, then ramps the speed target
down linearly. It also commands zero at the measured braking distance. Actual
stopping distance depends on speed tracking and available motor braking torque;
this does not command hydraulic brakes.

## Jack mode and existing wheel feedback

Stages 2 and 3 use integrated mean absolute wheel travel. Neither requires an
OMS connection. OMS can also be disabled at startup with `oms/enabled=false`;
this is optional for bench testing.

Measured motor RPMs arrive in `hytech.inv1_dynamics` through `hytech.inv4_dynamics`,
field `actual_speed_rpm`, mapped to FL, FR, RL, RR. These are recorded/streamed
alongside `hytech.drivebrain_speed_set_input` and
`hytech.drivebrain_torque_lim_input`. Plot all four measured and requested RPMs.
The conversion uses the existing 11.86 gearbox ratio and 0.4064 m wheel diameter.
The prior requirement for fresh RPMs from every inverter remains in place.

## Tests

Run the dependency-free profile, optical odometry, heading and remote-mission simulation tests:

```sh
cmake -S tests/launch -B build-launch-profile
cmake --build build-launch-profile
ctest --test-dir build-launch-profile --output-on-failure
```

For a native full build with normal dependencies configured, enable
`-DBUILD_LAUNCH_TESTS=ON` and run CTest. This additionally tests protobuf-to-state
mapping, startup controller selection, actual Foxglove stage/enable updates,
Xbox events through the Linux joystick reader, opposite-direction steering,
centering/deadzone/limits, OMS independence on jacks, zero output when disabled,
ground-distance braking, RSS/EBS snapshots, remote permission/Finished handshakes, latched STOP/timeout/abort behavior, blocked physical launch, joystick disconnect, original teleop index/settings,
and steering-only operation with trigger inputs and no drive feedback. The integration test binds port 5555;
stop other local Drivebrain instances first. Hardware speed scaling, mounting,
steering polarity and tracking still require physical verification.
