# Launch controller jack test

This is a temporary Drivebrain speed-profile test for the autonomous acceleration
controller. It uses the existing VCR Drivebrain/mode-4 drivetrain command path;
it does not implement autonomous mission authorization or hydraulic brakes.

In Foxglove's Parameters panel:

1. Leave `launchcontroller/testing_enabled=false` and
   `launchcontroller/brake_requested=false`.
2. Set `drivebraincontrollerinterface/controller_index=1` to select LaunchController
   (index 0 remains LoadCellTorqueController). Selection holds the Kraken at zero
   steering angle, taking priority over `kraken_comms/test_mode` and its angle.
   The existing steering encoder initialization and motor-reset fault behavior apply.
3. Configure the parameters below, put the drivetrain in ready-to-drive with VCR
   accepting Drivebrain commands, and ensure all four wheel speed measurements
   are present and stopped.
4. Change `launchcontroller/testing_enabled` from false to true for one run.
   At the acceleration distance, braking starts automatically. To brake earlier,
   set `launchcontroller/brake_requested=true` or set `testing_enabled=false`.
5. After stopping, set both booleans false, then toggle testing true for another run.
   Changing controllers discards a pending start; select launch before toggling.

| Parameter under `launchcontroller/` | Default | Meaning |
| --- | --- | --- |
| `acceleration_mps2` | 0.5 | Linear speed target slope, m/s² |
| `accel_distance_m` | 5.0 | Measured equivalent wheel travel before braking |
| `braking_distance_m` | 5.0 | Travel from braking entry to zero target speed |
| `max_speed_mps` | 2.0 | Speed target cap; hold this speed until accel distance |
| `max_torque_nm` | 2.0 | Per-motor speed-loop torque limit during acceleration |
| `max_regen_torque_nm` | 1.0 | Per-motor speed-loop torque limit during braking/hold |

Numeric settings are captured at the start of a run. Distances, acceleration,
speed and torque limits must be finite and positive. Torque limits cannot exceed
21 Nm propulsion / 10 Nm braking, and the speed cap cannot exceed 20,000 motor RPM.
A saved true flag never starts a run at boot. Missing/stale feedback, lost ready-to-drive,
invalid state, nonfinite speed or a control-loop gap over 100 ms faults the run;
it requires a new false-to-true edge after recovery. Starting requires every wheel
within 0.05 m/s of stopped. Braking finishes only when all four wheels are stopped.

The target follows `v = acceleration_mps2 * elapsed_time`, capped at max speed,
following the time-based ramp in the firmware's
[SimpleLaunchController](https://github.com/hytech-racing/vehicle-firmware/blob/main/VCR/lib/systems/include/controllers/SimpleLaunchController.h).
This test starts at zero speed. Braking captures the mean absolute measured wheel
speed `v_entry`, calculates `deceleration = v_entry² / (2 * braking_distance_m)`,
then ramps the speed target down linearly. The target becomes zero at the measured
braking distance even if the wheels lag the profile. This is inverter speed control:
actual stopping distance depends on torque limits and tracking; it is not a
closed-loop vehicle-position stop or a physical-brake command.

On jacks, distance is **virtual wheel travel**, from trapezoidal integration of the
mean absolute speed of all four wheels. Ground speed from the VectorNav would
remain zero. Motor RPM is converted through the existing 11.86 gearbox ratio and
0.4064 m wheel diameter. Ground testing will need a vehicle-speed/slip estimate.

Measured wheel feedback already arrives as `hytech.inv1_dynamics` through
`hytech.inv4_dynamics`, field `actual_speed_rpm`, mapped respectively to
FL, FR, RL, RR. These CAN messages are published to Foxglove and recorded in MCAP,
alongside `hytech.drivebrain_speed_set_input` and
`hytech.drivebrain_torque_lim_input`. Plot all four measured RPMs against the four
requested RPMs. The new freshness check requires every inverter measurement within
200 ms; VCR ready-to-drive expires after 500 ms. These software mappings are tested;
physical wheel identity, direction and speed tracking still need verification on jacks.

Run the dependency-free profile simulation:

```sh
cmake -S tests/launch -B build-launch-profile
cmake --build build-launch-profile
ctest --test-dir build-launch-profile --output-on-failure
```

For a native full build with the project's normal dependencies configured, enable
`-DBUILD_LAUNCH_TESTS=ON` and run CTest. This also tests real protobuf-to-StateTracker
wheel mapping, independent freshness expiry, controller selection, and real Foxglove
parameter toggles for launch, braking and fault recovery. The integration test binds
the local Foxglove server on port 5555, so stop other local Drivebrain instances first.
