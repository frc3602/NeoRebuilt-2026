# Architecture

## Overview

The robot code is organized around a small set of mechanism subsystems plus one coordinating class:

- `Drivetrain`
- `Turret`
- `Shooter`
- `Pivot`
- `Intake`
- `Spindexer`
- `Limelight_Pose`
- `SuperStructure`

`RobotContainer` creates the shared subsystem objects, binds controller buttons, and owns the auto chooser.

`Robot` runs the command scheduler and pushes telemetry to Elastic.

## Major Roles

### `Drivetrain`

- Owns the CTRE swerve drivetrain.
- Provides the estimated robot pose used by turret aiming, shooter distance calculation, and autonomous.
- Configures PathPlanner.

### `Turret`

- Uses Motion Magic position control.
- Aims robot-relative with:
  - `0` = front
  - positive = clockwise
  - negative = counterclockwise
  - `180` = rear
- Avoids crossing through the front seam when choosing a movement path.
- Can aim at the hub using pose, translational lead, and rotational lead.
- Can switch to an alliance pass corner while in center field.

### `Shooter`

- Uses Motion Magic velocity control.
- Computes distance to the current alliance hub from drivetrain pose.
- Uses a simple distance-to-speed model that can be tuned later.

### `Pivot`

- Uses Motion Magic position control for the intake pivot.
- Holds a setpoint internally, so it does not need a default command to keep position.

### `Intake`

- Uses Motion Magic velocity control for the intake roller.
- Provides forward, reverse, and stop actions.

### `Spindexer`

- Uses Motion Magic velocity control for:
  - spindexer motor
  - receiver motor
- Receiver speed is kept faster than the spindexer but slower than the shooter target speed.

### `Limelight_Pose`

- Reads Limelight data.
- Chooses acceptable vision measurements.
- Supplies accepted vision updates to the drivetrain estimator.

### `SuperStructure`

- Combines mechanism actions into higher-level scoring/intake/autonomous commands.
- Keeps `RobotContainer` button bindings simpler.

## Autonomous Flow

Autonomous command selection comes from the chooser in `RobotContainer`.

The `SuperStructure` now provides auton-friendly commands such as:

- prepare tracked shot
- prepare failsafe shot
- wait until ready
- feed a shot
- intake and stage
- stow

These are intended to be used directly in command sequences or PathPlanner named commands.
