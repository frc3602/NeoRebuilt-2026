# Subsystems

## `Drivetrain`

Purpose:

- swerve driving
- PathPlanner integration
- estimated pose source

Key notes:

- Pose comes from the CTRE drivetrain state.
- Accepted measurements from `Limelight_Pose` are fused into the estimator each loop.
- PathPlanner loads GUI robot settings first and falls back to the generated drivetrain config if needed.

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
- can aim at the alliance tower from pose
- adds translational and rotational lead offsets for moving shots
- switches to an alliance pass corner in center field

Useful commands:

- `rearPresetCommand()`
- `leftPresetCommand()`
- `aimAtHubWithMotionCompCommand()`
- `alignCommand()`

## `Shooter`

Purpose:

- Motion Magic velocity control for the flywheel

Key behavior:

- uses drivetrain pose to calculate distance to the current alliance hub
- predicts distance ahead based on robot motion before choosing the target velocity
- turns distance into a shooter target velocity with a tunable model

Useful commands:

- `setVelocityCommand(...)`
- `runDistanceBasedVelocityCommand()`
- `updateDistanceBasedVelocityOnceCommand()`
- `stopCommand()`

Current implementation note:

- The live shooter Slot0 gains are currently configured inside `Shooter.java`.

## `Pivot`

Purpose:

- Motion Magic position control for the intake pivot

Key behavior:

- stores and maintains a pivot setpoint without relying on a default command
- exposes fixed raise and drop commands used by teleop and autonomous code

Useful commands:

- `dumbDropIntake()`
- `dumbRaiseIntake()`
- `setPivotPositionCommand(...)`

## `Climber`

Purpose:

- Motion Magic position control for the climber

Key behavior:

- initializes the TalonFX rotor position to `0` on startup
- uses rotor encoder rotations directly for setpoints and telemetry
- keeps two main presets:
  - down = `0`
  - up = `Constants.ClimberConstants.kClimberUpRotations`
- can be disabled entirely with the climber feature flag

Useful commands:

- `raiseCommand()`
- `lowerCommand()`
- `setClimberPositionCommand(...)`
- `stopCommand()`

## `Intake`

Purpose:

- Motion Magic velocity control for the intake roller

Key behavior:

- runs forward for collecting fuel
- runs in reverse for clearing fuel

Useful commands:

- `intakeForwardCommand()`
- `intakeReverseCommand()`
- `stopCommand()`

## `Spindexer`

Purpose:

- Motion Magic velocity control for fuel staging and feeding

Owned motors:

- spindexer motor
- receiver motor

Key behavior:

- receiver speed is derived from the commanded spindexer speed
- receiver is faster than spindexer but slower than the shooter target speed
- target velocities are reapplied in `periodic()` so the motors keep following the latest commanded values
- current draw and measured speeds are exposed for telemetry
- there is no automatic jam detection or auto-reverse logic in the current implementation

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
- evaluates both MegaTag1 and MegaTag2 estimates
- filters stale, weak, or unreasonable frames
- exposes every accepted measurement for drivetrain fusion
- publishes a selected vision pose for dashboards
- publishes detailed left and right decision telemetry to SmartDashboard for tuning
