# Architecture

## Overview

The robot code is organized around a small set of subsystems plus one coordinating class:

- `Drivetrain`
- `Turret`
- `Shooter`
- `Pivot`
- `Intake`
- `Spindexer`
- `Climber`
- `Limelight_Pose`
- `SuperStructure`

`RobotContainer` creates the shared subsystem objects, binds controller buttons, registers PathPlanner named commands, and owns the autonomous chooser.

`Robot` runs the command scheduler and updates:

- `ElasticTelemetry`
- `AutonFieldPreview`
- `RobotPoseFieldView`

## Major Roles

### `Drivetrain`

- Owns the CTRE swerve drivetrain.
- Provides the estimated robot pose used by turret aiming, shooter distance calculation, and autonomous.
- Configures PathPlanner.
- Fuses accepted Limelight measurements into the pose estimator.

### `Turret`

- Uses Motion Magic position control.
- Aims robot-relative with:
  - `0` = front
  - `180` = rear
  - positive = clockwise
  - negative = counterclockwise
- Avoids crossing through the front seam when choosing a movement path.
- Can aim at the alliance tower from pose with translational and rotational lead.
- Can switch to an alliance pass corner while in center field.

### `Shooter`

- Uses Motion Magic velocity control.
- Computes distance to the current alliance hub from drivetrain pose.
- Uses a distance-to-speed model and motion-compensated lookahead based on robot motion.

### `Pivot`

- Uses Motion Magic position control for the intake pivot.
- Holds a setpoint internally, so it does not need a default command to keep position.

### `Intake`

- Uses Motion Magic velocity control for the intake roller.
- Provides forward, reverse, and stop actions for fuel handling.

### `Spindexer`

- Uses Motion Magic velocity control for:
  - the spindexer motor
  - the receiver motor
- Derives receiver speed from the commanded spindexer speed using a fixed ratio.
- Reapplies the active velocity targets every loop.

### `Climber`

- Uses Motion Magic position control for the climber.
- Exposes simple up and down presets.
- Can be left disabled with the climber feature flag if the hardware is not installed.

### `Limelight_Pose`

- Reads both Limelights.
- Evaluates MegaTag1 and MegaTag2 estimates for each camera.
- Rejects stale or weak frames and chooses accepted measurements.
- Publishes a selected vision pose for dashboards and accepted measurements for drivetrain fusion.

### `SuperStructure`

- Combines mechanism actions into higher-level shooting, intake, and autonomous commands.
- Keeps `RobotContainer` bindings and PathPlanner named commands simpler.

## Autonomous Flow

Autonomous command selection comes from the chooser in `RobotContainer`.

`RobotContainer` also registers PathPlanner named commands that map to `SuperStructure` helpers such as:

- tracked-shot preparation
- failsafe-shot preparation
- timed feed commands
- intake and staging
- stow

Those helpers are intended to be used directly in PathPlanner autos and command sequences.
