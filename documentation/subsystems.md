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

Current tuning note:

- Motion Magic speed tuning was increased from cruise `20.0` / accel `40.0` / jerk `200.0`
- Previous turret values were cruise `22.0` / accel `44.0` / jerk `220.0`
- Previous turret values were cruise `24.0` / accel `48.0` / jerk `240.0`
- Previous turret values were cruise `28.0` / accel `56.0` / jerk `280.0`
- Previous turret values were cruise `35.0` / accel `70.0` / jerk `350.0`
- Previous turret values were cruise `43.75` / accel `87.5` / jerk `437.5`
- Previous turret values were cruise `54.6875` / accel `109.375` / jerk `546.875`
- Previous turret values were cruise `68.0` / accel `137.0` / jerk `684.0`
- Previous turret values were cruise `85.0` / accel `171.0` / jerk `855.0`
- Previous turret values were cruise `85.0` / accel `214.0` / jerk `1069.0`
- Previous turret values were cruise `85.0` / accel `268.0` / jerk `1336.0`
- Current turret values are cruise `85.0` / accel `335.0` / jerk `1670.0`
- If more speed is needed later, adjust cruise first, then acceleration, then jerk in `Constants.TurretConstants`

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

## `Climber`

Purpose:

- Motion Magic position control for the climber

Key behavior:

- initializes the TalonFX rotor position to `0` on startup
- uses rotor encoder rotations directly for setpoints and telemetry
- keeps two main presets:
  - down = `0`
  - up = current tuned value in `Constants.ClimberConstants.kClimberUpRotations`

Useful commands:

- `raiseCommand()`
- `lowerCommand()`
- `setClimberPositionCommand(...)`
- `stopCommand()`

Current tuning note:

- Climber Motion Magic is currently using cruise `27.0` / accel `54.0` / jerk `270.0`
- Those values were increased in three 50% steps from cruise `8.0` / accel `16.0` / jerk `80.0`
- If the climber becomes too aggressive, reduce acceleration first, then cruise velocity, then jerk

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
- when commanded to run, a jam is detected if feed velocity stays low while either motor current is high
- jam detection waits briefly after startup and requires the condition to persist before reacting
- on a detected jam, both motors reverse at reduced speed for `0.35` seconds and then automatically resume the commanded direction

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
