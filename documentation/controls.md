# Controls

This file describes the current controller bindings from `RobotContainer`.

## Operator Controller

### `B` hold

- Runs the full tracked shot while held.
- While held:
  - turret aims at the live alliance target with motion compensation
  - shooter updates to a distance-based target velocity
  - spindexer begins feeding after the shooter-ready gate and tracked-shot feed delay

### `Right Bumper` hold

- Runs the full fallback shot while held.
- While held:
  - turret moves to the rear preset
  - shooter runs at the current fixed fallback target velocity
  - spindexer begins feeding once the turret and shooter are both ready

### `Y` hold

- Reverses the shooter, feed path, and intake to clear fuel from the system.

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

### `B`

- Snaps the drivetrain pose to the current accepted Limelight pose when pressed.
- If both Limelights are accepted, the snap uses a trust-weighted blend of their translations.
- The drivetrain keeps its current heading, so the gyro remains the source of rotation.

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

### `Y`

- No binding currently.

## Driver Rumble

The driver controller rumbles while:

- the operator is holding `B`
- and the tracked shot is fully ready to fire

In the current code, "fully ready" means the turret is at target and the shooter is near target speed.

This is meant to act as a simple "ready to send" signal.
