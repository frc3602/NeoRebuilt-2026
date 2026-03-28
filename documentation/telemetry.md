# Telemetry

The robot publishes telemetry to the `Elastic` NetworkTables tree.

Telemetry publishing in `ElasticTelemetry` is intentionally throttled to avoid excessive update load.

## Elastic Topics

### `Elastic/Status`

- `RobotMode`
- `Alliance`
- `DriverStationAttached`
- `Enabled`
- `Autonomous`
- `Teleop`
- `EStopped`
- `MatchTimeSeconds`
- `BatteryVoltage`

### `Elastic/Drive`

- `Pose`
- `PoseXMeters`
- `PoseYMeters`
- `HeadingDegrees`

### `Elastic/Vision`

- `HasVisionPose`
- `SelectedCamera`
- `Pose`
- `PoseXMeters`
- `PoseYMeters`
- `HeadingDegrees`
- `XYStdDev`
- `ThetaStdDev`

These are the currently selected vision values, not the full left and right camera debug set.

### `Elastic/Auto`

- `ShooterReady`
- `TurretAndShooterReady`
- `SelectedAuto`
- `PreviewStatus`

### `Elastic/Turret`

- `MeasuredAngleDegrees`
- `RequestedAngleDegrees`
- `AngleErrorDegrees`
- `AtTarget`
- `HubAimDegrees`
- `CompensatedAimDegrees`
- `InCenterField`
- `AlignMode`
- `AlignTargetXMeters`
- `AlignTargetYMeters`

### `Elastic/Shooter`

- `DistanceToHubMeters`
- `DistanceToHubFeet`
- `TargetVelocityRPS`
- `MeasuredVelocityRPS`
- `VelocityErrorRPS`
- `VelocityMagnitudeErrorRPS`
- `NearTargetSigned`
- `NearTargetMagnitude`

### `Elastic/Pivot`

- `MeasuredAngleDegrees`
- `SetpointAngleDegrees`

### `Elastic/Climber`

- `EncoderRotations`
- `EncoderTargetRotations`
- `IsAtPosition`

### `Elastic/Intake`

- `TargetVelocityRPS`
- `MeasuredVelocityRPS`

### `Elastic/Spindexer`

- `SpindexerTargetVelocityRPS`
- `SpindexerMeasuredVelocityRPS`
- `ReceiverTargetVelocityRPS`
- `ReceiverMeasuredVelocityRPS`
- `SpindexerCurrentAmps`
- `ReceiverCurrentAmps`
- `FeedingCommanded`

## Dashboard Widgets

The robot also publishes these `Field2d` and chooser widgets through SmartDashboard:

- `Elastic/Auton Field`
- `Elastic/Auto Chooser`
- `Elastic/Robot Pose Field`

## Additional Vision Diagnostics

`Limelight_Pose` publishes more detailed camera-by-camera tuning data to SmartDashboard, not to Elastic.

Those keys are grouped under:

- `Vision/Selected/*`
- `Vision/Left/*`
- `Vision/Right/*`
- `Vision/Drive/*`

That SmartDashboard data is where the full per-camera acceptance, rejection, quality, and pose-jump diagnostics currently live.
