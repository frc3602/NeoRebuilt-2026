package frc.robot;

import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import frc.robot.Constants.ShooterConstants;
import frc.robot.subsystems.Intake;
import frc.robot.subsystems.Pivot;
import frc.robot.subsystems.Shooter;
import frc.robot.subsystems.Spindexer;
import frc.robot.subsystems.Turret;

public class SuperStructure {
    private static final double kManualFeedVelocityRotationsPerSecond = 56.125;
    private static final double kFailsafeShooterVelocityRotationsPerSecond =
        ShooterConstants.kShooterTargetVelocityRotationsPerSecond;
    private static final double kTrackedShotReadyToleranceRotationsPerSecond = 5.0;
    private static final double kAutonFeedTimeSeconds = 0.35;
    private static final double kAutonIntakeTimeSeconds = 1.0;

    private final Turret turret;
    private final Shooter shooter;
    private final Spindexer spindexer;
    private final Pivot pivot;
    private final Intake intake;

    public SuperStructure(
            Turret turret,
            Shooter shooter,
            Spindexer spindexer,
            Pivot pivot,
            Intake intake) {
        this.turret = turret;
        this.shooter = shooter;
        this.spindexer = spindexer;
        this.pivot = pivot;
        this.intake = intake;
    }

    public Command manualFeedReverse() {
        return Commands.startEnd(
            () -> {
                shooter.setVelocityRotationsPerSecond(
                    ShooterConstants.kShooterReverseVelocityRotationsPerSecond);
                spindexer.setFeedVelocityRotationsPerSecond(kManualFeedVelocityRotationsPerSecond);
                intake.intakeReverse();
            },
            () -> {
                shooter.stop();
                spindexer.stop();
                intake.stop();
            },
            shooter,
            spindexer,
            intake);
    }

    public Command shootFailsafe() {
        final boolean[] hasStartedFeeding = {false};
        return Commands.parallel(
            Commands.run(() -> turret.setTurretAngleDegrees(180.0), turret),
            Commands.run(() -> shooter.setVelocityRotationsPerSecond(
                kFailsafeShooterVelocityRotationsPerSecond), shooter),
            Commands.runEnd(
                () -> {
                    if (!hasStartedFeeding[0]
                            && isTurretAndShooterReadyForShot()) {
                        hasStartedFeeding[0] = true;
                    }

                    if (hasStartedFeeding[0]) {
                        spindexer.feedForward();
                    }
                },
                () -> {
                    hasStartedFeeding[0] = false;
                    spindexer.stop();
                },
                spindexer));
    }

    public Command prepareFailsafeShot() {
        return Commands.parallel(
            turret.rearPresetCommand(),
            shooter.setVelocityCommand(kFailsafeShooterVelocityRotationsPerSecond));
    }

    public Command stopShoot() {
        return Commands.parallel(
            shooter.stopCommand(),
            spindexer.stopCommand());
    }

    public Command trackAllianceTower() {
        return shootTrackedLerpShot();
    }

    public boolean isShooterReadyForShot() {
        return shooter.isNearTargetVelocityMagnitude(kTrackedShotReadyToleranceRotationsPerSecond);
    }

    public boolean isTurretAndShooterReadyForShot() {
        return turret.atTarget() && isShooterReadyForShot();
    }

    public boolean isShooterNearTargetVelocitySigned() {
        return shooter.isNearTargetVelocity(kTrackedShotReadyToleranceRotationsPerSecond);
    }

    public boolean isTrackedLerpShotReady() {
        return isTurretAndShooterReadyForShot();
    }

    public boolean isTrackedLerpShotFeedReady() {
        return isShooterReadyForShot();
    }

    public Command shootTrackedLerpShot() {
        final boolean[] hasStartedFeeding = {false};
        return Commands.parallel(
            turret.aimAtHubWithMotionCompCommand(),
            shooter.runDistanceBasedVelocityCommand(),
            Commands.runEnd(
                () -> {
                    if (!hasStartedFeeding[0] && isTrackedLerpShotFeedReady()) {
                        hasStartedFeeding[0] = true;
                    }

                    if (hasStartedFeeding[0]) {
                        spindexer.feedForward();
                    }
                },
                () -> {
                    hasStartedFeeding[0] = false;
                    spindexer.stop();
                },
                spindexer));
    }

    public Command fixedShotCommand(double shooterVelocityRotationsPerSecond) {
        return shooter.setVelocityCommand(shooterVelocityRotationsPerSecond);
    }

    public Command stopShooterOnly() {
        return shooter.stopCommand();
    }

    public Command turretRearPreset() {
        return turret.rearPresetCommand();
    }

    public Command turretLeftPreset() {
        return turret.leftPresetCommand();
    }

    public Command turretLeftCornerPreset() {
        return turret.rearLeftCornerPresetCommand();
    }

    public Command turretRightCornerPreset() {
        return turret.rearRightCornerPresetCommand();
    }

    public Command dropPivot() {
        return pivot.dumbDropIntake();
    }

    public Command raisePivot() {
        return pivot.dumbRaiseIntake();
    }

    public Command runIntake() {
        return intake.intakeForwardCommand();
    }

    public Command stopIntake() {
        return intake.stopCommand();
    }

    /* ---------------- Autonomous Commands ---------------- */

    public Command autonPrepareTrackedShot() {
        return Commands.parallel(
            turret.aimAtHubWithMotionCompOnceCommand(),
            shooter.updateDistanceBasedVelocityOnceCommand());
    }

    public Command autonPrepareFailsafeShot() {
        return Commands.parallel(
            turret.rearPresetCommand(),
            shooter.setVelocityCommand(kFailsafeShooterVelocityRotationsPerSecond));
    }

    public Command autonWaitForTrackedShotReady() {
        return Commands.waitUntil(this::isTrackedLerpShotReady);
    }

    public Command autonFeedShot() {
        return Commands.sequence(
            spindexer.feedForwardCommand(),
            Commands.waitSeconds(kAutonFeedTimeSeconds),
            spindexer.stopCommand());
    }

    public Command autonShootTrackedShot() {
        return Commands.sequence(
            autonPrepareTrackedShot(),
            autonWaitForTrackedShotReady(),
            autonFeedShot());
    }

    public Command autonShootFailsafeShot() {
        return Commands.sequence(
            autonPrepareFailsafeShot(),
            Commands.waitUntil(this::isTurretAndShooterReadyForShot),
            autonFeedShot());
    }

    public Command autonRunIntake() {
        return Commands.parallel(
            pivot.dumbDropIntake(),
            Commands.sequence(
                intake.intakeForwardCommand(),
                Commands.waitSeconds(kAutonIntakeTimeSeconds),
                intake.stopCommand()));
    }

    public Command autonIntakeAndStage() {
        return Commands.parallel(
            pivot.dumbDropIntake(),
            intake.intakeForwardCommand(),
            spindexer.feedForwardCommand());
    }

    public Command autonStopGamePiecePath() {
        return Commands.parallel(
            intake.stopCommand(),
            spindexer.stopCommand());
    }

    public Command autonStow() {
        return Commands.parallel(
            pivot.dumbRaiseIntake(),
            intake.stopCommand(),
            spindexer.stopCommand(),
            shooter.stopCommand());
    }
}
