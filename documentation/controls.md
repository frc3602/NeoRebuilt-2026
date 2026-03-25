# Controls

This file describes the current controller bindings from `RobotContainer`.

## Operator Controller

### `B` hold

- Runs the full tracked shot.
- While held:
  - turret aligns to the live target
  - shooter updates to distance-based speed
  - spindexer starts automatically once the shot is ready and keeps feeding until release

### `Right Bumper` hold

- Runs the full failsafe shot.
- While held:
  - turret moves to the rear preset
  - shooter runs at the fixed failsafe speed
  - spindexer starts automatically once the shot is ready and keeps feeding until release

### `Y` hold

- Reverses the shooter, receiver/spindexer, and intake for clearing or unjamming.

### `Left Bumper`

- Raises the intake pivot to the up position.

### `POV Up`

- Sends the climber to the up setpoint.

### `POV Down`

- Sends the climber to the down setpoint (`0`).

## Driver Controller

### `Left Stick`

- Drives the robot translationally in field-centric control.

### `Right Stick X`

- Rotates the robot.

### `Right Bumper`

- Drops the pivot to the intake position.

### `Left Bumper`

- Starts the intake on press.
- Stops the intake on release.

### `Right Trigger`

- Reserved.
- Currently reports:
  - turbo mode not implemented
  - normal mode restore not implemented

### `A`

- Reserved.
- Currently reports that drive polarity change is not implemented.

### `X` hold

- Brakes the swerve drive.

### `Y` hold

- Points the swerve modules toward the left stick direction.

## Driver Rumble

The driver controller rumbles while:

- the operator is holding `B`
- and the tracked shot is actually ready to fire

This is meant to act as a simple “ready to send” signal.
