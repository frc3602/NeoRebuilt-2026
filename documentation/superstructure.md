# SuperStructure

`SuperStructure` is the high-level command layer that combines the mechanism subsystems.

It exists to keep button bindings and autonomous sequences readable.

## Teleop-Oriented Commands

### `manualFeedReverse()`

- Reverses the shooter, feed path, and intake at fixed manual velocities.

### `shootFailsafe()`

- Runs the fallback shot while held.
- Commands:
  - rear turret preset
  - fixed shooter target velocity
  - automatic spindexer feed once turret and shooter are ready

### `prepareFailsafeShot()`

- One-shot setup for the failsafe shot.
- Does not feed by itself.

### `stopShoot()`

- Stops the shooter and spindexer.

### `trackAllianceTower()`

- Currently acts as a legacy alias for the full tracked-shot command.
- It does not only move the turret.

### `isTrackedLerpShotReady()`

- Returns true when:
  - turret is at target
  - shooter is close to its active target velocity

### `isTrackedLerpShotFeedReady()`

- Returns true when the shooter is close to its active target velocity.
- In the current code, this feed gate does not also require the turret to be at target.

### `shootTrackedLerpShot()`

- Runs the tracked shot while held.
- Commands:
  - turret alignment with motion compensation
  - distance-based shooter speed
  - automatic spindexer feed after the shooter-ready gate and fixed feed delay

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
- `raisePivot()`
- `runIntake()`
- `stopIntake()`

## Autonomous-Oriented Commands

### `autonPrepareTrackedShot()`

- One-shot autonomous setup for a tracked shot:
  - align turret once
  - update shooter speed once from distance

### `autonPrepareFailsafeShot()`

- One-shot autonomous setup for a fixed fallback shot.

### `autonWaitForTrackedShotReady()`

- Waits for the tracked-shot feed gate, then waits the fixed feed delay.
- In the current code, that feed gate is based on shooter readiness, not turret-at-target.

### `autonFeedShot()`

- Feeds fuel for a fixed amount of time, then stops the spindexer.

### `autonShootTrackedShot()`

- Full autonomous tracked shot sequence:
  - keep aiming
  - wait for the feed gate
  - feed for a fixed time

### `autonShootFailsafeShot()`

- Full autonomous failsafe shot sequence:
  - prepare
  - wait for shooter readiness
  - feed

### `autonRunIntake()`

- Drops the pivot and runs the intake for a fixed amount of time.

### `autonIntakeAndStage()`

- Starts pivot drop, intake, and spindexer together.
- Intended for continuous fuel collection and staging until interrupted.

### `autonStopGamePiecePath()`

- Stops intake and spindexer.

### `autonStow()`

- Raises pivot and stops:
  - intake
  - spindexer
  - shooter
