package frc.robot;

import java.util.function.BooleanSupplier;

import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import frc.robot.Constants.ShooterConstants;
import frc.robot.Constants.SpindexerConstants;
import frc.robot.Constants.TurretConstants;
import frc.robot.subsystems.Intake;
import frc.robot.subsystems.Pivot;
import frc.robot.subsystems.Shooter;
import frc.robot.subsystems.Spindexer;
import frc.robot.subsystems.Turret;

public class SuperStructure {
    private static final double kManualFeedVelocityRotationsPerSecond =
        SpindexerConstants.scaleSpindexerVelocityRotationsPerSecond(56.125);
    private static final double kFailsafeShooterVelocityRotationsPerSecond =
        ShooterConstants.kShooterTargetVelocityRotationsPerSecond;
    private static final double kTrackedShotReadyToleranceRotationsPerSecond = 2.5;
    private static final double kTrackedShotFeedToleranceRotationsPerSecond = 2.5;
    private static final double kTrackedShotFeedDelaySeconds = 0.50;
    private static final double kAutonFeedTimeSeconds = 10;
    private static final double kAutonOutpostShootTimeSeconds = 10.0;
    private static final double kAutonReverseSpindexerTimeSeconds = 0.5;
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
        return Commands.parallel(
            Commands.run(() -> turret.setTurretAngleDegrees(180.0), turret),
            Commands.run(() -> shooter.setVelocityRotationsPerSecond(
                kFailsafeShooterVelocityRotationsPerSecond), shooter),
            waitUntilReadyThenContinuousFeedCommand(this::isTurretAndShooterReadyToFeedShot));
    }

    public Command shootFailsafePass() {
        return Commands.parallel(
            Commands.run(() -> turret.setTurretAngleDegrees(180.0), turret),
            Commands.run(() -> shooter.setVelocityRotationsPerSecond(
                kFailsafeShooterVelocityRotationsPerSecond * 1.1), shooter),
            waitUntilReadyThenContinuousFeedCommand(this::isTurretAndShooterReadyToFeedShot));
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
        return shootTrackedBallisticShot();
    }

    public boolean isShooterReadyForShot() {
        return shooter.isNearTargetVelocityMagnitude(kTrackedShotReadyToleranceRotationsPerSecond);
    }

    private boolean isShooterReadyToFeedShot() {
        return shooter.isNearTargetVelocityMagnitude(
            kTrackedShotFeedToleranceRotationsPerSecond);
    }

    private boolean isTurretReadyForShot() {
        return turret.isNearRequestedAngleDegrees(TurretConstants.kTurretShotToleranceDegrees);
    }

    public boolean isTurretAndShooterReadyForShot() {
        return isTurretReadyForShot() && isShooterReadyForShot();
    }

    private boolean isTurretAndShooterReadyToFeedShot() {
        return isTurretReadyForShot() && isShooterReadyToFeedShot();
    }

    public boolean isShooterNearTargetVelocitySigned() {
        return shooter.isNearTargetVelocity(kTrackedShotReadyToleranceRotationsPerSecond);
    }

    public boolean isTrackedBallisticShotReady() {
        return isTurretAndShooterReadyForShot();
    }

    public boolean isTrackedBallisticShotFeedReady() {
        return isTurretAndShooterReadyToFeedShot();
    }

    public Command shootTrackedBallisticShot() {
        return Commands.parallel(
            turret.aimAtHubWithMotionCompCommand(),
            shooter.runDistanceBasedVelocityCommand(),
            waitUntilReadyDelayThenContinuousFeedCommand(this::isTrackedBallisticShotFeedReady))
            .finallyDo(spindexer::stop);
    }

    public Command shootTrackedBallisticShotWithFeedMultiplier() {
        return Commands.parallel(
            turret.aimAtHubWithMotionCompCommand(),
            shooter.runDistanceBasedVelocityCommand(),
            waitUntilReadyDelayThenFeedCommand(
                this::isTrackedBallisticShotFeedReady,
                continuousFeedWithTrackedShotVelocityMultiplierCommand()))
            .finallyDo(spindexer::stop);
    }

    public boolean isTrackedPassCornerShotReady() {
        return turret.isOutsideAllianceZone()
            && isTurretAndShooterReadyForShot();
    }

    public boolean isTrackedPassCornerShotFeedReady() {
        return turret.isOutsideAllianceZone()
            && isTurretAndShooterReadyToFeedShot();
    }

    public Command shootTrackedPassCornerShot() {
        return Commands.parallel(
            turret.aimAtPassCornerWithMotionCompCommand(),
            Commands.run(
                () -> shooter.updateVelocityForTarget(turret.getCurrentPassCornerTranslation()),
                shooter),
            waitUntilReadyDelayThenContinuousFeedCommand(this::isTrackedPassCornerShotFeedReady))
            .finallyDo(() -> {
                spindexer.stop();
                shooter.stop();
            });
    }

    private Command continuousFeedCommand() {
        return Commands.run(
            () -> spindexer.feedForShooterVelocityRotationsPerSecond(
                shooter.getTargetVelocityRotationsPerSecond()),
            spindexer);
    }

    private Command continuousFeedWithTrackedShotVelocityMultiplierCommand() {
        return Commands.run(
            () -> spindexer.feedForShooterVelocityRotationsPerSecondWithTrackedShotMultiplier(
                shooter.getTargetVelocityRotationsPerSecond()),
            spindexer);
    }

    private Command waitUntilReadyThenFeedCommand(BooleanSupplier isFeedReady, Command feedCommand) {
        return Commands.sequence(
            Commands.waitUntil(isFeedReady),
            feedCommand);
    }

    private Command waitUntilReadyThenContinuousFeedCommand(BooleanSupplier isFeedReady) {
        return waitUntilReadyThenFeedCommand(isFeedReady, continuousFeedCommand());
    }

    private Command waitUntilReadyDelayThenFeedCommand(BooleanSupplier isFeedReady, Command feedCommand) {
        return Commands.sequence(
            Commands.waitUntil(isFeedReady),
            Commands.waitSeconds(kTrackedShotFeedDelaySeconds),
            Commands.waitUntil(isFeedReady),
            feedCommand);
    }

    private Command waitUntilReadyDelayThenContinuousFeedCommand(BooleanSupplier isFeedReady) {
        return waitUntilReadyDelayThenFeedCommand(isFeedReady, continuousFeedCommand());
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

    public Command partialRaisePivot() {
        return pivot.dumbPartialRaiseIntake();
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
        return Commands.sequence(
            Commands.waitUntil(this::isTrackedBallisticShotFeedReady),
            Commands.waitSeconds(kTrackedShotFeedDelaySeconds),
            Commands.waitUntil(this::isTrackedBallisticShotFeedReady));
    }

    public Command autonFeedShot() {
        return Commands.deadline(
            Commands.waitSeconds(kAutonFeedTimeSeconds),
            waitUntilReadyThenContinuousFeedCommand(this::isTurretAndShooterReadyToFeedShot))
            .finallyDo(spindexer::stop);
    }

    public Command autonReverseSpindexerHalfSecond() {
        return Commands.startEnd(
            spindexer::feedReverse,
            spindexer::stop,
            spindexer)
            .withTimeout(kAutonReverseSpindexerTimeSeconds);
    }

    public Command autonShootTrackedShot() {
        return Commands.sequence(
            Commands.deadline(
                autonWaitForTrackedShotReady(),
                turret.aimAtHubWithMotionCompCommand(),
                Commands.run(shooter::updateVelocityForCurrentDistance, shooter)),
            Commands.deadline(
                Commands.waitSeconds(kAutonFeedTimeSeconds),
                turret.aimAtHubWithMotionCompCommand(),
                Commands.run(shooter::updateVelocityForCurrentDistance, shooter),
                waitUntilReadyThenContinuousFeedCommand(this::isTrackedBallisticShotFeedReady))
                .finallyDo(() -> {
                    spindexer.stop();
                    shooter.stop();
                }));
    }

    public Command autonOutpostShootCommand() {
        return Commands.sequence(
            Commands.deadline(
                Commands.sequence(
                    Commands.waitUntil(this::isTrackedPassCornerShotFeedReady),
                    Commands.waitSeconds(kTrackedShotFeedDelaySeconds)),
                turret.aimAtPassCornerWithMotionCompCommand(),
                Commands.run(
                    () -> shooter.updateVelocityForTarget(turret.getCurrentPassCornerTranslation()),
                    shooter)),
            Commands.deadline(
                Commands.waitSeconds(kAutonOutpostShootTimeSeconds),
                turret.aimAtPassCornerWithMotionCompCommand(),
                Commands.run(
                    () -> shooter.updateVelocityForTarget(turret.getCurrentPassCornerTranslation()),
                    shooter),
                waitUntilReadyThenContinuousFeedCommand(this::isTrackedPassCornerShotFeedReady))
                .finallyDo(() -> {
                    spindexer.stop();
                    shooter.stop();
                }));
    }

    public Command autonShootFailsafeShot() {
        return Commands.sequence(
            autonPrepareFailsafeShot(),
            Commands.waitUntil(this::isTurretAndShooterReadyToFeedShot),
            autonFeedShot());
    }

    public Command autonRunIntake() {
        return Commands.run(intake::intakeForward, intake);
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
