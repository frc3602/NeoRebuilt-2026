# Subsystems

## `Drivetrain`

Purpose:

- Swerve driving
- PathPlanner integration
- estimated pose source

Key notes:

- Pose comes from the CTRE drivetrain state.
- Vision measurements from `Limelight_Pose` are fused into the estimator.

## `Turret`

Purpose:

- Motion Magic position control for turret aiming

Angle convention:

- `0` = front of robot
- `180` = back of robot
- positive = clockwise
- negative = counterclockwise

Key behavior:

- avoids crossing the front seam when moving to a setpoint
- can aim at the hub from pose
- adds translational and rotational lead offsets
- switches to an alliance pass corner in center field

Useful commands:

- `rearPresetCommand()`
- `alignCommand()`
- `alignOnceCommand()`
- `aimAtHubWithMotionCompCommand()`

## `Shooter`

Purpose:

- Motion Magic velocity control for the flywheel

Key behavior:

- uses drivetrain pose to calculate distance to the current alliance hub
- turns distance into a shooter target velocity with a tunable model

Useful commands:

- `setVelocityCommand(...)`
- `runDistanceBasedVelocityCommand()`
- `updateDistanceBasedVelocityOnceCommand()`
- `stopCommand()`

## `Pivot`

Purpose:

- Motion Magic position control for the intake pivot

Key behavior:

- stores and maintains a pivot setpoint without relying on a default command

Useful commands:

- `dumbDropIntake()`
- `dumbRaiseIntake()`
- `setPivotPositionCommand(...)`

## `Intake`

Purpose:

- Motion Magic velocity control for the intake roller

Useful commands:

- `intakeForwardCommand()`
- `intakeReverseCommand()`
- `stopCommand()`

## `Spindexer`

Purpose:

- Motion Magic velocity control for note staging and feeding

Owned motors:

- spindexer motor
- receiver motor

Key behavior:

- receiver speed is derived from spindexer speed
- receiver is faster than spindexer but slower than the shooter target speed

Useful commands:

- `feedForwardCommand()`
- `feedReverseCommand()`
- `setFeedVelocityCommand(...)`
- `stopCommand()`

## `Limelight_Pose`

Purpose:

- vision pose filtering and selection

Key behavior:

- reads both Limelights
- filters stale/bad frames
- exposes accepted measurements for drivetrain fusion
- publishes selected vision pose info for telemetry
