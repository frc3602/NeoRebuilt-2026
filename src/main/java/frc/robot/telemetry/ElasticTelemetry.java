package frc.robot.telemetry;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.util.Units;
import edu.wpi.first.networktables.BooleanPublisher;
import edu.wpi.first.networktables.DoublePublisher;
import edu.wpi.first.networktables.NetworkTable;
import edu.wpi.first.networktables.NetworkTableInstance;
import edu.wpi.first.networktables.StringPublisher;
import edu.wpi.first.networktables.StructPublisher;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.RobotController;
import edu.wpi.first.wpilibj.Timer;
import frc.robot.RobotContainer;
import frc.robot.SuperStructure;
import frc.robot.subsystems.Climber;
import frc.robot.subsystems.Intake;
import frc.robot.subsystems.Limelight_Pose;
import frc.robot.subsystems.Drivetrain;
import frc.robot.subsystems.Pivot;
import frc.robot.subsystems.Shooter;
import frc.robot.subsystems.Spindexer;
import frc.robot.subsystems.Turret;

public class ElasticTelemetry {
  private static final double kPublishPeriodSeconds = 0.05;

  private final Limelight_Pose m_limelightPose = Limelight_Pose.getInstance();
  private double m_lastPublishTimeSeconds = Double.NEGATIVE_INFINITY;

  private final StringPublisher m_robotModePublisher;
  private final StringPublisher m_alliancePublisher;
  private final BooleanPublisher m_dsAttachedPublisher;
  private final BooleanPublisher m_enabledPublisher;
  private final BooleanPublisher m_autonomousPublisher;
  private final BooleanPublisher m_teleopPublisher;
  private final BooleanPublisher m_estoppedPublisher;
  private final DoublePublisher m_matchTimePublisher;
  private final DoublePublisher m_batteryVoltagePublisher;
  private final DoublePublisher m_poseXPublisher;
  private final DoublePublisher m_poseYPublisher;
  private final DoublePublisher m_headingDegreesPublisher;
  private final StructPublisher<Pose2d> m_posePublisher;
  private final BooleanPublisher m_visionAvailablePublisher;
  private final StringPublisher m_visionCameraPublisher;
  private final DoublePublisher m_visionXPublisher;
  private final DoublePublisher m_visionYPublisher;
  private final DoublePublisher m_visionHeadingDegreesPublisher;
  private final DoublePublisher m_visionXYStdDevPublisher;
  private final DoublePublisher m_visionThetaStdDevPublisher;
  private final StructPublisher<Pose2d> m_visionPosePublisher;
  private final BooleanPublisher m_autoShooterReadyPublisher;
  private final BooleanPublisher m_autoTurretAndShooterReadyPublisher;
  private final DoublePublisher m_turretMeasuredAnglePublisher;
  private final DoublePublisher m_turretRequestedAnglePublisher;
  private final DoublePublisher m_turretAngleErrorPublisher;
  private final BooleanPublisher m_turretAtTargetPublisher;
  private final DoublePublisher m_turretHubAimPublisher;
  private final DoublePublisher m_turretCompensatedAimPublisher;
  private final BooleanPublisher m_turretCenterFieldPublisher;
  private final StringPublisher m_turretAlignModePublisher;
  private final DoublePublisher m_turretAlignTargetXPublisher;
  private final DoublePublisher m_turretAlignTargetYPublisher;
  private final DoublePublisher m_shooterDistancePublisher;
  private final DoublePublisher m_shooterDistanceFeetPublisher;
  private final DoublePublisher m_shooterTargetVelocityPublisher;
  private final DoublePublisher m_shooterMeasuredVelocityPublisher;
  private final DoublePublisher m_shooterVelocityErrorPublisher;
  private final DoublePublisher m_shooterVelocityMagnitudeErrorPublisher;
  private final BooleanPublisher m_shooterNearTargetSignedPublisher;
  private final BooleanPublisher m_shooterNearTargetMagnitudePublisher;
  private final DoublePublisher m_pivotMeasuredAnglePublisher;
  private final DoublePublisher m_pivotSetpointPublisher;
  private final DoublePublisher m_climberPositionPublisher;
  private final DoublePublisher m_climberTargetPositionPublisher;
  private final BooleanPublisher m_climberAtTargetPublisher;
  private final DoublePublisher m_intakeTargetVelocityPublisher;
  private final DoublePublisher m_intakeMeasuredVelocityPublisher;
  private final DoublePublisher m_spindexerTargetVelocityPublisher;
  private final DoublePublisher m_spindexerMeasuredVelocityPublisher;
  private final DoublePublisher m_receiverTargetVelocityPublisher;
  private final DoublePublisher m_receiverMeasuredVelocityPublisher;
  private final DoublePublisher m_spindexerCurrentPublisher;
  private final DoublePublisher m_receiverCurrentPublisher;
  private final BooleanPublisher m_spindexerFeedingPublisher;

  public ElasticTelemetry() {
    NetworkTable elasticTable = NetworkTableInstance.getDefault().getTable("Elastic");
    NetworkTable statusTable = elasticTable.getSubTable("Status");
    NetworkTable driveTable = elasticTable.getSubTable("Drive");
    NetworkTable visionTable = elasticTable.getSubTable("Vision");
    NetworkTable autoTable = elasticTable.getSubTable("Auto");
    NetworkTable turretTable = elasticTable.getSubTable("Turret");
    NetworkTable shooterTable = elasticTable.getSubTable("Shooter");
    NetworkTable pivotTable = elasticTable.getSubTable("Pivot");
    NetworkTable climberTable = elasticTable.getSubTable("Climber");
    NetworkTable intakeTable = elasticTable.getSubTable("Intake");
    NetworkTable spindexerTable = elasticTable.getSubTable("Spindexer");

    m_robotModePublisher = statusTable.getStringTopic("RobotMode").publish();
    m_alliancePublisher = statusTable.getStringTopic("Alliance").publish();
    m_dsAttachedPublisher = statusTable.getBooleanTopic("DriverStationAttached").publish();
    m_enabledPublisher = statusTable.getBooleanTopic("Enabled").publish();
    m_autonomousPublisher = statusTable.getBooleanTopic("Autonomous").publish();
    m_teleopPublisher = statusTable.getBooleanTopic("Teleop").publish();
    m_estoppedPublisher = statusTable.getBooleanTopic("EStopped").publish();
    m_matchTimePublisher = statusTable.getDoubleTopic("MatchTimeSeconds").publish();
    m_batteryVoltagePublisher = statusTable.getDoubleTopic("BatteryVoltage").publish();

    m_poseXPublisher = driveTable.getDoubleTopic("PoseXMeters").publish();
    m_poseYPublisher = driveTable.getDoubleTopic("PoseYMeters").publish();
    m_headingDegreesPublisher = driveTable.getDoubleTopic("HeadingDegrees").publish();
    m_posePublisher = driveTable.getStructTopic("Pose", Pose2d.struct).publish();

    m_visionAvailablePublisher = visionTable.getBooleanTopic("HasVisionPose").publish();
    m_visionCameraPublisher = visionTable.getStringTopic("SelectedCamera").publish();
    m_visionXPublisher = visionTable.getDoubleTopic("PoseXMeters").publish();
    m_visionYPublisher = visionTable.getDoubleTopic("PoseYMeters").publish();
    m_visionHeadingDegreesPublisher = visionTable.getDoubleTopic("HeadingDegrees").publish();
    m_visionXYStdDevPublisher = visionTable.getDoubleTopic("XYStdDev").publish();
    m_visionThetaStdDevPublisher = visionTable.getDoubleTopic("ThetaStdDev").publish();
    m_visionPosePublisher = visionTable.getStructTopic("Pose", Pose2d.struct).publish();

    m_autoShooterReadyPublisher = autoTable.getBooleanTopic("ShooterReady").publish();
    m_autoTurretAndShooterReadyPublisher = autoTable.getBooleanTopic("TurretAndShooterReady").publish();

    m_turretMeasuredAnglePublisher = turretTable.getDoubleTopic("MeasuredAngleDegrees").publish();
    m_turretRequestedAnglePublisher = turretTable.getDoubleTopic("RequestedAngleDegrees").publish();
    m_turretAngleErrorPublisher = turretTable.getDoubleTopic("AngleErrorDegrees").publish();
    m_turretAtTargetPublisher = turretTable.getBooleanTopic("AtTarget").publish();
    m_turretHubAimPublisher = turretTable.getDoubleTopic("HubAimDegrees").publish();
    m_turretCompensatedAimPublisher = turretTable.getDoubleTopic("CompensatedAimDegrees").publish();
    m_turretCenterFieldPublisher = turretTable.getBooleanTopic("InCenterField").publish();
    m_turretAlignModePublisher = turretTable.getStringTopic("AlignMode").publish();
    m_turretAlignTargetXPublisher = turretTable.getDoubleTopic("AlignTargetXMeters").publish();
    m_turretAlignTargetYPublisher = turretTable.getDoubleTopic("AlignTargetYMeters").publish();

    m_shooterDistancePublisher = shooterTable.getDoubleTopic("DistanceToHubMeters").publish();
    m_shooterDistanceFeetPublisher = shooterTable.getDoubleTopic("DistanceToHubFeet").publish();
    m_shooterTargetVelocityPublisher = shooterTable.getDoubleTopic("TargetVelocityRPS").publish();
    m_shooterMeasuredVelocityPublisher = shooterTable.getDoubleTopic("MeasuredVelocityRPS").publish();
    m_shooterVelocityErrorPublisher = shooterTable.getDoubleTopic("VelocityErrorRPS").publish();
    m_shooterVelocityMagnitudeErrorPublisher = shooterTable.getDoubleTopic("VelocityMagnitudeErrorRPS").publish();
    m_shooterNearTargetSignedPublisher = shooterTable.getBooleanTopic("NearTargetSigned").publish();
    m_shooterNearTargetMagnitudePublisher = shooterTable.getBooleanTopic("NearTargetMagnitude").publish();

    m_pivotMeasuredAnglePublisher = pivotTable.getDoubleTopic("MeasuredAngleDegrees").publish();
    m_pivotSetpointPublisher = pivotTable.getDoubleTopic("SetpointAngleDegrees").publish();
    m_climberPositionPublisher = climberTable.getDoubleTopic("EncoderRotations").publish();
    m_climberTargetPositionPublisher = climberTable.getDoubleTopic("EncoderTargetRotations").publish();
    m_climberAtTargetPublisher = climberTable.getBooleanTopic("IsAtPosition").publish();

    m_intakeTargetVelocityPublisher = intakeTable.getDoubleTopic("TargetVelocityRPS").publish();
    m_intakeMeasuredVelocityPublisher = intakeTable.getDoubleTopic("MeasuredVelocityRPS").publish();

    m_spindexerTargetVelocityPublisher = spindexerTable.getDoubleTopic("SpindexerTargetVelocityRPS").publish();
    m_spindexerMeasuredVelocityPublisher = spindexerTable.getDoubleTopic("SpindexerMeasuredVelocityRPS").publish();
    m_receiverTargetVelocityPublisher = spindexerTable.getDoubleTopic("ReceiverTargetVelocityRPS").publish();
    m_receiverMeasuredVelocityPublisher = spindexerTable.getDoubleTopic("ReceiverMeasuredVelocityRPS").publish();
    m_spindexerCurrentPublisher = spindexerTable.getDoubleTopic("SpindexerCurrentAmps").publish();
    m_receiverCurrentPublisher = spindexerTable.getDoubleTopic("ReceiverCurrentAmps").publish();
    m_spindexerFeedingPublisher = spindexerTable.getBooleanTopic("FeedingCommanded").publish();
  }

  public void update(RobotContainer robotContainer) {
    double nowSeconds = Timer.getFPGATimestamp();
    if (nowSeconds - m_lastPublishTimeSeconds < kPublishPeriodSeconds) {
      return;
    }
    m_lastPublishTimeSeconds = nowSeconds;

    Drivetrain drivetrain = robotContainer.getDrivetrain();
    Turret turret = robotContainer.getTurret();
    Shooter shooter = robotContainer.getShooter();
    Pivot pivot = robotContainer.getPivot();
    Climber climber = robotContainer.getClimber();
    Intake intake = robotContainer.getIntake();
    Spindexer spindexer = robotContainer.getSpindexer();
    SuperStructure superStructure = robotContainer.getSuperStructure();
    Pose2d pose = drivetrain.getEstimatedPose();

    m_robotModePublisher.set(getRobotMode());
    m_alliancePublisher.set(
        DriverStation.getAlliance().map(Enum::name).orElse("Unknown"));
    m_dsAttachedPublisher.set(DriverStation.isDSAttached());
    m_enabledPublisher.set(DriverStation.isEnabled());
    m_autonomousPublisher.set(DriverStation.isAutonomousEnabled());
    m_teleopPublisher.set(DriverStation.isTeleopEnabled());
    m_estoppedPublisher.set(DriverStation.isEStopped());
    m_matchTimePublisher.set(DriverStation.getMatchTime());
    m_batteryVoltagePublisher.set(RobotController.getBatteryVoltage());

    m_poseXPublisher.set(pose.getX());
    m_poseYPublisher.set(pose.getY());
    m_headingDegreesPublisher.set(pose.getRotation().getDegrees());
    m_posePublisher.set(pose);

    if (m_limelightPose.poseUpdateAvailable && m_limelightPose.poseCamEstimate != null) {
      Pose2d visionPose = m_limelightPose.poseCamEstimate.pose;
      m_visionAvailablePublisher.set(true);
      m_visionCameraPublisher.set(m_limelightPose.getSelectedCameraName());
      m_visionXPublisher.set(visionPose.getX());
      m_visionYPublisher.set(visionPose.getY());
      m_visionHeadingDegreesPublisher.set(visionPose.getRotation().getDegrees());
      m_visionXYStdDevPublisher.set(m_limelightPose.poseUpdateXYTrustFactor);
      m_visionThetaStdDevPublisher.set(m_limelightPose.poseUpdateRotTrustFactor);
      m_visionPosePublisher.set(visionPose);
    } else {
      m_visionAvailablePublisher.set(false);
      m_visionCameraPublisher.set("None");
      m_visionXPublisher.set(Double.NaN);
      m_visionYPublisher.set(Double.NaN);
      m_visionHeadingDegreesPublisher.set(Double.NaN);
      m_visionXYStdDevPublisher.set(Double.NaN);
      m_visionThetaStdDevPublisher.set(Double.NaN);
    }

    m_autoShooterReadyPublisher.set(superStructure.isShooterReadyForShot());
    m_autoTurretAndShooterReadyPublisher.set(superStructure.isTurretAndShooterReadyForShot());

    m_turretMeasuredAnglePublisher.set(turret.getTurretAngleDegrees());
    m_turretRequestedAnglePublisher.set(turret.getRequestedAngleDegrees());
    m_turretAngleErrorPublisher.set(turret.getAngleErrorDegrees());
    m_turretAtTargetPublisher.set(turret.atTarget());
    m_turretHubAimPublisher.set(turret.calculateHubAimAngleDegrees());
    m_turretCompensatedAimPublisher.set(turret.calculateCompensatedHubAimAngleDegrees());
    m_turretCenterFieldPublisher.set(turret.isInCenterField());
    m_turretAlignModePublisher.set(turret.getAlignTargetMode());
    m_turretAlignTargetXPublisher.set(turret.getCurrentAlignTargetTranslation().getX());
    m_turretAlignTargetYPublisher.set(turret.getCurrentAlignTargetTranslation().getY());

    double shooterDistanceMeters = shooter.getDistanceToHubMeters();
    m_shooterDistancePublisher.set(shooterDistanceMeters);
    m_shooterDistanceFeetPublisher.set(Units.metersToFeet(shooterDistanceMeters));
    m_shooterTargetVelocityPublisher.set(shooter.getTargetVelocityRotationsPerSecond());
    m_shooterMeasuredVelocityPublisher.set(shooter.getMeasuredVelocityRotationsPerSecond());
    m_shooterVelocityErrorPublisher.set(
        shooter.getMeasuredVelocityRotationsPerSecond()
            - shooter.getTargetVelocityRotationsPerSecond());
    m_shooterVelocityMagnitudeErrorPublisher.set(
        Math.abs(shooter.getMeasuredVelocityRotationsPerSecond())
            - Math.abs(shooter.getTargetVelocityRotationsPerSecond()));
    m_shooterNearTargetSignedPublisher.set(superStructure.isShooterNearTargetVelocitySigned());
    m_shooterNearTargetMagnitudePublisher.set(superStructure.isShooterReadyForShot());

    m_pivotMeasuredAnglePublisher.set(pivot.getRightPosition());
    m_pivotSetpointPublisher.set(pivot.getPivotSetpointDegrees());
    m_climberPositionPublisher.set(climber.getClimberPositionRotations());
    m_climberTargetPositionPublisher.set(climber.getTargetPositionRotations());
    m_climberAtTargetPublisher.set(climber.atTarget());

    m_intakeTargetVelocityPublisher.set(intake.getTargetVelocityRotationsPerSecond());
    m_intakeMeasuredVelocityPublisher.set(intake.getMeasuredVelocityRotationsPerSecond());

    m_spindexerTargetVelocityPublisher.set(spindexer.getSpindexerTargetVelocityRotationsPerSecond());
    m_spindexerMeasuredVelocityPublisher.set(spindexer.getSpindexerMeasuredVelocityRotationsPerSecond());
    m_receiverTargetVelocityPublisher.set(spindexer.getReceiverTargetVelocityRotationsPerSecond());
    m_receiverMeasuredVelocityPublisher.set(spindexer.getReceiverMeasuredVelocityRotationsPerSecond());
    m_spindexerCurrentPublisher.set(spindexer.getSpindexerCurrentAmps());
    m_receiverCurrentPublisher.set(spindexer.getReceiverCurrentAmps());
    m_spindexerFeedingPublisher.set(
        Math.abs(spindexer.getSpindexerTargetVelocityRotationsPerSecond()) > 1e-6
            || Math.abs(spindexer.getReceiverTargetVelocityRotationsPerSecond()) > 1e-6);
  }

  private String getRobotMode() {
    if (DriverStation.isEStopped()) {
      return "EStopped";
    }
    if (DriverStation.isAutonomousEnabled()) {
      return "Autonomous";
    }
    if (DriverStation.isTeleopEnabled()) {
      return "Teleop";
    }
    if (DriverStation.isTestEnabled()) {
      return "Test";
    }
    return "Disabled";
  }
}
