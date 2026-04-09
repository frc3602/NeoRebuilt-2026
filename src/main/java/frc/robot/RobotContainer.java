// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

import com.pathplanner.lib.commands.PathPlannerAuto;

import static edu.wpi.first.units.Units.MetersPerSecond;
import static edu.wpi.first.units.Units.RadiansPerSecond;
import static edu.wpi.first.units.Units.RotationsPerSecond;

import com.ctre.phoenix6.swerve.SwerveModule.DriveRequestType;
import com.ctre.phoenix6.swerve.SwerveRequest;
import com.pathplanner.lib.auto.AutoBuilder;
import com.pathplanner.lib.auto.NamedCommands;
import edu.wpi.first.wpilibj.GenericHID.RumbleType;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import edu.wpi.first.wpilibj2.command.button.CommandXboxController;
import edu.wpi.first.wpilibj.smartdashboard.SendableChooser;
import edu.wpi.first.wpilibj2.command.button.Trigger;
import frc.robot.Constants.OIConstants;
import frc.robot.generated.TunerConstants;
import frc.robot.subsystems.Climber;
import frc.robot.subsystems.Drivetrain;
import frc.robot.subsystems.Intake;
import frc.robot.subsystems.Limelight_Pose;
import frc.robot.subsystems.Pivot;
import frc.robot.subsystems.Shooter;
import frc.robot.subsystems.Spindexer;
import frc.robot.subsystems.Turret;
import java.util.ArrayList;
import java.util.Collections;
import java.util.List;

public class RobotContainer {
    private static final String kPreferredAutoName = "Basic Center Auto";

    private static final String kLeftTrenchDepotAutoName = "Left Trench, Depot";
    private static final String kShootTestAutoName = "Shoot Test";
    private static final String kRightTrenchOutpostAutoName = "Right Trench, Outpost";
    private static final String kRightTrenchCenterRightAlliance = "Right Trench, Center, Right Alliance";
    private static final String kCenterHubOutpost = "Center Hub, Outpost";
    private static final String kCenterHubDepot = "Center Hub, Depot";
    private static final String kBasicCenterAutoName = "Basic Center Auto";
    
    private double MaxSpeed = 1.0 * TunerConstants.kSpeedAt12Volts.in(MetersPerSecond); // kSpeedAt12Volts desired top                                                                                   // speed
    private double MaxAngularRate = RotationsPerSecond.of(0.75).in(RadiansPerSecond); // 3/4 of a rotation per second                                                                             // max angular velocity
    private final SwerveRequest.FieldCentric drive =
        new SwerveRequest.FieldCentric()
            .withDeadband(MaxSpeed * 0.1)
            .withRotationalDeadband(MaxAngularRate * 0.1)
            .withDriveRequestType(DriveRequestType.OpenLoopVoltage);

  private final Drivetrain m_drivetrain = TunerConstants.createDrivetrain();
  @SuppressWarnings("unused")
  private final Limelight_Pose m_limelightPose = Limelight_Pose.getInstance();
  private final Turret m_turret = new Turret(m_drivetrain);
  private final Shooter m_shooter = new Shooter(m_drivetrain);
  private final Pivot m_pivot = new Pivot();
  private final Climber m_climber = new Climber();
  private final Intake m_intake = new Intake();
  private final Spindexer m_spindexer = new Spindexer();
  private final SuperStructure m_superStructure =
      new SuperStructure(m_turret, m_shooter, m_spindexer, m_pivot, m_intake);
  private final CommandXboxController m_driverController =
      new CommandXboxController(OIConstants.kDriverControllerPort);
  private final CommandXboxController m_operatorController =
      new CommandXboxController(OIConstants.kOperatorControllerPort);
  private final SendableChooser<String> m_autoChooser = new SendableChooser<>();
  private final SwerveRequest.SwerveDriveBrake brake = new SwerveRequest.SwerveDriveBrake();
    private final SwerveRequest.PointWheelsAt point = new SwerveRequest.PointWheelsAt();


  public RobotContainer() {
    m_drivetrain.configPathplanner();
    configureNamedCommands();
    configureAutonomousChooser();
    configureBindings();
  }

  private void configureBindings() {

      m_drivetrain.setDefaultCommand(
                // Drivetrain will execute this command periodically
                m_drivetrain.applyRequest(() -> drive.withVelocityX(-m_driverController.getLeftY() * MaxSpeed) // Drive forward with
                                                                                                  // negative Y
                                                                                                  // (forward)
                        .withVelocityY(-m_driverController.getLeftX() * MaxSpeed) // Drive left with negative X (left)
                        .withRotationalRate(-m_driverController.getRightX() * MaxAngularRate) // Drive counterclockwise with
                                                                                    // negative X (left)

    ));
    
    Trigger scoreHeld = m_operatorController.b();

    m_operatorController.y()
        .whileTrue(m_superStructure.manualFeedReverse())
        .onFalse(m_superStructure.stopShoot());

    m_operatorController.leftBumper().onTrue(m_superStructure.partialRaisePivot()).onFalse(m_superStructure.raisePivot());
    m_operatorController.povUp().onTrue(m_climber.raiseCommand());
    m_operatorController.povDown().onTrue(m_climber.lowerCommand());

    m_operatorController.rightBumper()
        .whileTrue(m_superStructure.shootFailsafe())
        .onFalse(m_superStructure.stopShoot());

    scoreHeld
        .whileTrue(m_superStructure.shootTrackedBallisticShot())
        .onFalse(m_superStructure.stopShoot());

    new Trigger(() -> m_operatorController.getHID().getBButton() && m_superStructure.isTrackedBallisticShotReady())
        .whileTrue(driverShotReadyRumble());

    m_driverController.rightBumper().onTrue(m_superStructure.dropPivot());
    m_driverController.leftBumper()
        .onTrue(m_superStructure.runIntake())
        .onFalse(m_superStructure.stopIntake());

    m_operatorController.rightTrigger()
        .and(m_turret::isOutsideAllianceZone)
        .whileTrue(m_superStructure.shootTrackedPassCornerShot())
        .onFalse(m_superStructure.stopShoot());

    m_driverController.a().onTrue(reportUnimplementedDriveMode("Drive polarity change not implemented yet"));

    m_driverController.x().whileTrue(m_drivetrain.applyRequest(() -> brake));

    
  }

  private void configureAutonomousChooser() {
    List<String> availableAutoNames = new ArrayList<>(AutoBuilder.getAllAutoNames());
    Collections.sort(availableAutoNames);
    List<String> chooserAutoNames = new ArrayList<>();

    if (availableAutoNames.isEmpty()) {
      m_autoChooser.setDefaultOption("None", "");
      DriverStation.reportWarning("No PathPlanner autos were found.", false);
      return;
    }

    if (availableAutoNames.remove(kPreferredAutoName)) {
      m_autoChooser.setDefaultOption(kPreferredAutoName, kPreferredAutoName);
      chooserAutoNames.add(kPreferredAutoName);
    } else {
      String defaultAutoName = availableAutoNames.remove(0);
      m_autoChooser.setDefaultOption(defaultAutoName, defaultAutoName);
      chooserAutoNames.add(defaultAutoName);
      DriverStation.reportWarning(
          "Preferred auto '" + kPreferredAutoName + "' was not found in PathPlanner autos.", false);
    }


    addRequestedAutoOption(availableAutoNames, chooserAutoNames, kLeftTrenchDepotAutoName);
    addRequestedAutoOption(availableAutoNames, chooserAutoNames, kShootTestAutoName);
    addRequestedAutoOption(availableAutoNames, chooserAutoNames, kRightTrenchOutpostAutoName);
    addRequestedAutoOption(availableAutoNames, chooserAutoNames, kRightTrenchCenterRightAlliance);
    addRequestedAutoOption(availableAutoNames, chooserAutoNames, kCenterHubOutpost);
    addRequestedAutoOption(availableAutoNames, chooserAutoNames, kCenterHubDepot);
    addRequestedAutoOption(availableAutoNames, chooserAutoNames, kBasicCenterAutoName);
  }

  private void addRequestedAutoOption(
      List<String> availableAutoNames, List<String> chooserAutoNames, String autoName) {
    if (chooserAutoNames.contains(autoName)) {
      return;
    }

    if (availableAutoNames.remove(autoName)) {
      m_autoChooser.addOption(autoName, autoName);
      chooserAutoNames.add(autoName);
      return;
    }

    if (!kPreferredAutoName.equals(autoName)) {
      DriverStation.reportWarning(
          "Requested chooser auto '" + autoName + "' was not found in PathPlanner autos.", false);
    }
  }

  private void configureNamedCommands() {
    NamedCommands.registerCommand("ManualFeedReverse", m_superStructure.manualFeedReverse());
    NamedCommands.registerCommand("ShootFailsafe", m_superStructure.shootFailsafe());
    NamedCommands.registerCommand("PrepareFailsafeShot", m_superStructure.prepareFailsafeShot());
    NamedCommands.registerCommand("StopShoot", m_superStructure.stopShoot());
    NamedCommands.registerCommand("TrackAllianceTower", m_superStructure.trackAllianceTower());
    NamedCommands.registerCommand(
        "ShootTrackedBallisticShot", m_superStructure.shootTrackedBallisticShot());
    NamedCommands.registerCommand(
        "ShootTrackedLerpShot", m_superStructure.shootTrackedBallisticShot());
    NamedCommands.registerCommand("StopShooterOnly", m_superStructure.stopShooterOnly());
    NamedCommands.registerCommand("TurretRearPreset", m_superStructure.turretRearPreset());
    NamedCommands.registerCommand("TurretLeftPreset", m_superStructure.turretLeftPreset());
    NamedCommands.registerCommand(
        "TurretLeftCornerPreset", m_superStructure.turretLeftCornerPreset());
    NamedCommands.registerCommand(
        "TurretRightCornerPreset", m_superStructure.turretRightCornerPreset());
    NamedCommands.registerCommand("DropPivot", m_superStructure.dropPivot());
    NamedCommands.registerCommand("RaisePivot", m_superStructure.raisePivot());
    NamedCommands.registerCommand("RunIntake", m_superStructure.runIntake());
    NamedCommands.registerCommand("StopIntake", m_superStructure.stopIntake());
    NamedCommands.registerCommand("ClimberUp", m_climber.raiseCommand());
    NamedCommands.registerCommand("ClimberDown", m_climber.lowerCommand());

    NamedCommands.registerCommand(
        "AutonPrepareTrackedShot", m_superStructure.autonPrepareTrackedShot());
    NamedCommands.registerCommand(
        "AutonPrepareFailsafeShot", m_superStructure.autonPrepareFailsafeShot());
    NamedCommands.registerCommand(
        "AutonWaitForTrackedShotReady", m_superStructure.autonWaitForTrackedShotReady());
    NamedCommands.registerCommand("AutonFeedShot", m_superStructure.autonFeedShot());
    NamedCommands.registerCommand(
        "AutonReverseSpindexerHalfSecond", m_superStructure.autonReverseSpindexerHalfSecond());
    NamedCommands.registerCommand(
        "AutonShootTrackedShot", m_superStructure.autonShootTrackedShot());
    NamedCommands.registerCommand(
        "AutonOutpostShootCommand", m_superStructure.autonOutpostShootCommand());
    NamedCommands.registerCommand(
        "AutonShootFailsafeShot", m_superStructure.autonShootFailsafeShot());
    NamedCommands.registerCommand("AutonRunIntake", m_superStructure.autonRunIntake());
    NamedCommands.registerCommand(
        "AutonIntakeAndStage", m_superStructure.autonIntakeAndStage());
    NamedCommands.registerCommand(
        "AutonStopGamePiecePath", m_superStructure.autonStopGamePiecePath());
    NamedCommands.registerCommand("AutonStow", m_superStructure.autonStow());
  }

  public Drivetrain getDrivetrain() {
    return m_drivetrain;
  }

  public Turret getTurret() {
    return m_turret;
  }

  public Shooter getShooter() {
    return m_shooter;
  }

  public Pivot getPivot() {
    return m_pivot;
  }

  public Intake getIntake() {
    return m_intake;
  }

  public Climber getClimber() {
    return m_climber;
  }

  public Spindexer getSpindexer() {
    return m_spindexer;
  }

  public SuperStructure getSuperStructure() {
    return m_superStructure;
  }

  public SendableChooser<String> getAutoChooser() {
    return m_autoChooser;
  }

  public String getSelectedAutoName() {
    return m_autoChooser.getSelected() == null ? "" : m_autoChooser.getSelected();
  }

  public Command getAutonomousCommand() {
    String autoName = getSelectedAutoName();
    if (autoName.isBlank()) {
      return Commands.print("No autonomous command configured");
    }

    if (!AutoBuilder.isConfigured()) {
      DriverStation.reportWarning(
          "Autonomous requested, but PathPlanner AutoBuilder is not configured.", false);
      return Commands.print("AutoBuilder not configured; skipping autonomous");
    }

    try {
      return new PathPlannerAuto(autoName);
    } catch (RuntimeException ex) {
      DriverStation.reportError(
          "Failed to create autonomous command '" + autoName + "': " + ex.getMessage(), false);
      return Commands.print("Failed to create autonomous command");
    }
  }

  private Command driverShotReadyRumble() {
    return Commands.startEnd(
        () -> m_driverController.getHID().setRumble(RumbleType.kBothRumble, 1.0),
        () -> m_driverController.getHID().setRumble(RumbleType.kBothRumble, 0.0));
  }

  private Command reportUnimplementedDriveMode(String message) {
    return Commands.runOnce(() -> DriverStation.reportWarning(message, false));
  }
}
