// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

import com.pathplanner.lib.commands.PathPlannerAuto;
import com.pathplanner.lib.auto.AutoBuilder;
import edu.wpi.first.wpilibj.GenericHID.RumbleType;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import edu.wpi.first.wpilibj2.command.button.CommandXboxController;
import edu.wpi.first.wpilibj.smartdashboard.SendableChooser;
import edu.wpi.first.wpilibj2.command.button.Trigger;
import frc.robot.Constants.OIConstants;
import frc.robot.generated.TunerConstants;
import frc.robot.subsystems.Drivetrain;
import frc.robot.subsystems.Intake;
import frc.robot.subsystems.Limelight_Pose;
import frc.robot.subsystems.Pivot;
import frc.robot.subsystems.Shooter;
import frc.robot.subsystems.Spindexer;
import frc.robot.subsystems.Turret;

public class RobotContainer {
  private final Drivetrain m_drivetrain = TunerConstants.createDrivetrain();
  @SuppressWarnings("unused")
  private final Limelight_Pose m_limelightPose = Limelight_Pose.getInstance();
  private final Turret m_turret = new Turret(m_drivetrain);
  private final Shooter m_shooter = new Shooter(m_drivetrain);
  private final Pivot m_pivot = new Pivot();
  private final Intake m_intake = new Intake();
  private final Spindexer m_spindexer = new Spindexer();
  private final SuperStructure m_superStructure =
      new SuperStructure(m_turret, m_shooter, m_spindexer, m_pivot, m_intake);
  private final CommandXboxController m_driverController =
      new CommandXboxController(OIConstants.kDriverControllerPort);
  private final CommandXboxController m_operatorController =
      new CommandXboxController(OIConstants.kOperatorControllerPort);
  private final SendableChooser<String> m_autoChooser = new SendableChooser<>();

  public RobotContainer() {
    m_drivetrain.configPathplanner();
    configureAutonomousChooser();
    configureBindings();
  }

  private void configureBindings() {
    Trigger scoreHeld = m_operatorController.b();

    m_operatorController.y()
        .whileTrue(m_superStructure.manualFeedReverse())
        .onFalse(m_superStructure.stopShoot());

    m_operatorController.rightBumper()
        .whileTrue(m_superStructure.shootFailsafe())
        .onFalse(m_superStructure.stopShoot());

    scoreHeld
        .whileTrue(m_superStructure.shootTrackedLerpShot())
        .onFalse(m_superStructure.stopShoot());

    new Trigger(() -> m_operatorController.getHID().getBButton() && m_superStructure.isTrackedLerpShotReady())
        .whileTrue(driverShotReadyRumble());

    m_driverController.rightBumper().onTrue(m_superStructure.dropPivot());
    m_driverController.leftBumper()
        .onTrue(m_superStructure.runIntake())
        .onFalse(m_superStructure.stopIntake());

    m_driverController.rightTrigger()
        .onTrue(reportUnimplementedDriveMode("Turbo mode not implemented yet"))
        .onFalse(reportUnimplementedDriveMode("Normal drive mode restore not implemented yet"));

    m_driverController.a().onTrue(reportUnimplementedDriveMode("Drive polarity change not implemented yet"));
  }

  private void configureAutonomousChooser() {
    m_autoChooser.setDefaultOption("None", "");

    for (String autoName : AutoBuilder.getAllAutoNames()) {
      m_autoChooser.addOption(autoName, autoName);
    }
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

  public Spindexer getSpindexer() {
    return m_spindexer;
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
