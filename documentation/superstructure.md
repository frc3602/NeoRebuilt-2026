# SuperStructure

`SuperStructure` is the high-level command layer that combines the mechanism subsystems.

It exists to keep button bindings and autonomous sequences readable.

## Teleop-Oriented Commands

### `manualFeedReverse()`

- Reverses the feed path at a fixed manual velocity.

### `shootFailsafe()`

- Runs the fixed fallback shot while held.
- Commands:
  - rear turret preset
  - fixed shooter speed
  - automatic spindexer feed once turret and shooter are ready

### `prepareFailsafeShot()`

- One-shot setup for the failsafe shot.
- Does not feed by itself.

### `stopShoot()`

- Stops the shooter and spindexer.

### `trackAllianceTower()`

- Runs the turret align command continuously.

### `isTrackedLerpShotReady()`

- Returns true when:
  - turret is at target
  - shooter is close to its active target velocity

### `shootTrackedLerpShot()`

- Runs the tracked shot while held.
- Commands:
  - turret align
  - distance-based shooter speed
  - automatic spindexer feed once ready

### `fixedShotCommand(double shooterVelocityRotationsPerSecond)`

- Spins the shooter to a manually specified fixed speed.

### `stopShooterOnly()`

- Stops only the shooter.

### Preset Helpers

- `turretRearPreset()`
- `turretLeftPreset()`
- `turretLeftCornerPreset()`
- `turretRightCornerPreset()`
- `dropPivot()`
- `runIntake()`
- `stopIntake()`

## Autonomous-Oriented Commands

### `autonPrepareTrackedShot()`

- One-shot auton setup for a tracked shot:
  - align turret once
  - update shooter speed once from distance

### `autonPrepareFailsafeShot()`

- One-shot auton setup for a fixed failsafe shot.

### `autonWaitForTrackedShotReady()`

- Waits until the tracked shot is ready.

### `autonFeedShot()`

- Feeds a shot for a fixed amount of time, then stops the spindexer.

### `autonShootTrackedShot()`

- Full autonomous tracked shot sequence:
  - prepare
  - wait ready
  - feed

### `autonShootFailsafeShot()`

- Full autonomous failsafe shot sequence:
  - prepare
  - wait ready
  - feed

### `autonRunIntake()`

- Drops the pivot and runs the intake for a fixed amount of time.

### `autonIntakeAndStage()`

- Starts pivot drop, intake, and spindexer together.
- Intended for continuous note collection/staging until interrupted.

### `autonStopGamePiecePath()`

- Stops intake and spindexer.

### `autonStow()`

- Raises pivot and stops:
  - intake
  - spindexer
  - shooter
