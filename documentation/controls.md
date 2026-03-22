# Controls

This file describes the current controller bindings from `RobotContainer`.

## Operator Controller

### `B` hold

- Runs the full tracked shot.
- While held:
  - turret aligns to the live target
  - shooter updates to distance-based speed
  - spindexer feeds automatically once the shot is ready

### `Right Bumper` hold

- Runs the full failsafe shot.
- While held:
  - turret moves to the rear preset
  - shooter runs at the fixed failsafe speed
  - spindexer feeds automatically once the shot is ready

### `Y` hold

- Reverses the feed path for clearing or unjamming.

## Driver Controller

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

## Driver Rumble

The driver controller rumbles while:

- the operator is holding `B`
- and the tracked shot is actually ready to fire

This is meant to act as a simple “ready to send” signal.
