# Shoot on the move

Right trigger keeps updating the turret bearing, hood angle, and flywheel RPM from one
shared drive-state sample per robot loop. Translation and rotation are both capped at
20% while the trigger is held. Normal limits return on release.

The solver subtracts turret field velocity times ball flight time from the target,
then repeats the lookup at the resulting effective distance. Turret velocity includes
robot translation and rotation around the offset pivot. Bearing feedforward follows
the compensated target. Measured field-velocity changes estimate translational acceleration;
measured angular-velocity changes estimate angular acceleration. Both predict position and
velocity at ball exit, 0.13 seconds ahead. Robot acceleration is not applied to the ball
after it exits. Turret angle conversion uses the same sampled heading as the solver.

`ShotAccelerationEstimator` uses a 0.04-second filter time constant and starting bounds
of 8 m/s² and 30 rad/s². These are tuning values, not measured limits. The estimate
resets after sample gaps over 0.1 seconds. Log `Shot/FieldAcceleration` and
`Shot/AngularAcceleration` while testing braking and direction changes. Filtering and
measurement latency remain; the predictor cannot anticipate future joystick changes.

The existing one-time battery snapshot remains: below 12.0 V on trigger press adds
10 hood degrees until release. The flywheel still needs 0.8 seconds continuously
within its speed tolerance before initial feed. Fuel transport pauses if aim is lost,
the solve fails to converge, or effective distance leaves the shot map (currently
1.0–3.4 m). Balls already past the transport rollers cannot be recalled.
After a large turret wraparound begins, the kicker and transport rollers are commanded
to stop during every shooting phase, including initial priming. They can resume once
the turret is within 15 degrees of the target, subject to the other feed conditions.
Already-released balls and mechanical coast-down cannot be undone by this interlock.

## Measurements needed

- `src/main/java/frc/robot/shooting/ShotMap.java`: the fourth column is exit-to-hub
  flight time in seconds. The current code table is the source of truth for calibration;
  entries have been updated during robot testing. Recheck interpolated rows whenever
  neighboring measurements change.
  Use slow-motion video at each distance, keeping the existing hood/RPM calibration.
- `MovingShotCalculator.kReleaseDelaySecs`: 0.13 seconds, measured from hopper to
  shooter exit. Do not use the 0.8-second flywheel
  settling time here: aiming continues updating throughout spin-up.
- Confirm `TurretConstants.kRobotToTurret`: currently 0.19685 m behind and 0.13335 m
  left of robot center, in WPILib coordinates. Verify field pose and heading first.
- Recheck flight time with the low-battery hood offset; the current table uses one
  flight-time curve for both battery states.

Start with stationary shots, then low-speed sideways travel, then toward/away travel,
then rotation. Log `Shot/VirtualGoal`, `Shot/ReleaseVelocity`,
`Shot/EffectiveDistanceMeters`, `Shot/TimeOfFlightSecs`, `Shot/Valid`, and
`BallPath/MovingShotFeedAllowed`. Flight times are code constants, not live dashboard
controls. This implementation has not been validated with physical moving shots.

Reference approach: [Mechanical Advantage LaunchCalculator](https://github.com/Mechanical-Advantage/RobotCode2026Public/blob/alpha-bot-turret/src/main/java/org/littletonrobotics/frc2026/subsystems/launcher/LaunchCalculator.java).
Calibration values from that robot were not used.
