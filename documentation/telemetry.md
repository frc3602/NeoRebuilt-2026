# Elastic Telemetry

The robot publishes telemetry to the `Elastic` NetworkTables tree.

Telemetry publishing is intentionally throttled in `ElasticTelemetry` to avoid excessive update load.

## Status Topics

Published under `Elastic/Status`:

- `RobotMode`
- `Alliance`
- `DriverStationAttached`
- `Enabled`
- `Autonomous`
- `Teleop`
- `EStopped`
- `MatchTimeSeconds`
- `BatteryVoltage`

## Drive Topics

Published under `Elastic/Drive`:

- `Pose`
- `PoseXMeters`
- `PoseYMeters`
- `HeadingDegrees`

## Vision Topics

Published under `Elastic/Vision`:

- `HasVisionPose`
- `SelectedCamera`
- `Pose`
- `PoseXMeters`
- `PoseYMeters`
- `HeadingDegrees`
- `XYStdDev`
- `ThetaStdDev`

## Turret Topics

Published under `Elastic/Turret`:

- `MeasuredAngleDegrees`
- `RequestedAngleDegrees`
- `AtTarget`
- `HubAimDegrees`
- `CompensatedAimDegrees`
- `InCenterField`
- `AlignMode`
- `AlignTargetXMeters`
- `AlignTargetYMeters`

## Shooter Topics

Published under `Elastic/Shooter`:

- `DistanceToHubMeters`
- `TargetVelocityRPS`
- `MeasuredVelocityRPS`

## Pivot Topics

Published under `Elastic/Pivot`:

- `MeasuredAngleDegrees`
- `SetpointAngleDegrees`

## Climber Topics

Published under `Elastic/Climber`:

- `EncoderRotations`
- `EncoderTargetRotations`
- `IsAtPosition`

## Intake Topics

Published under `Elastic/Intake`:

- `TargetVelocityRPS`
- `MeasuredVelocityRPS`

## Spindexer Topics

Published under `Elastic/Spindexer`:

- `SpindexerTargetVelocityRPS`
- `SpindexerMeasuredVelocityRPS`
- `ReceiverTargetVelocityRPS`
- `ReceiverMeasuredVelocityRPS`
