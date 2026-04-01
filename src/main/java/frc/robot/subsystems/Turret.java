package frc.robot.subsystems;

import com.ctre.phoenix6.configs.CurrentLimitsConfigs;
import com.ctre.phoenix6.configs.MotionMagicConfigs;
import com.ctre.phoenix6.configs.MotorOutputConfigs;
import com.ctre.phoenix6.configs.Slot0Configs;
import com.ctre.phoenix6.controls.MotionMagicVoltage;
import com.ctre.phoenix6.hardware.TalonFX;
import com.ctre.phoenix6.signals.NeutralModeValue;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.DriverStation.Alliance;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants.FieldConstants;
import frc.robot.Constants.ShooterConstants;
import frc.robot.Constants.TurretConstants;

public class Turret extends SubsystemBase {
    private final TalonFX turretMotor = new TalonFX(TurretConstants.kTurretMotorID);
    private final MotionMagicVoltage positionRequest = new MotionMagicVoltage(0);
    private final Drivetrain drivetrain;

    private double requestedAngleDegrees = TurretConstants.kTurretStartAngleDegrees;
    private double requestedUnwrappedAngleDegrees = TurretConstants.kTurretStartAngleDegrees;

    public Turret(Drivetrain drivetrain) {
        this.drivetrain = drivetrain;
        configureMotor();
        seedStartAngle();
        // setTurretAngleDegrees(TurretConstants.kTurretStartAngleDegrees);
    }

    public double getTurretAngleDegrees() {
        return normalizeSignedAngleDegrees(getTurretUnwrappedAngleDegrees());
    }

    public double getTurretUnwrappedAngleDegrees() {
        return rotorRotationsToDegrees(turretMotor.getRotorPosition().getValueAsDouble());
    }

    public double getRequestedAngleDegrees() {
        return requestedAngleDegrees;
    }

    public boolean atTarget() {
        return isNearRequestedAngleDegrees(TurretConstants.kTurretPositionToleranceDegrees);
    }

    public boolean isNearRequestedAngleDegrees(double toleranceDegrees) {
        return Math.abs(getAngleErrorDegrees()) <= toleranceDegrees;
    }

    public double getAngleErrorDegrees() {
        return requestedUnwrappedAngleDegrees - getTurretUnwrappedAngleDegrees();
    }

    public void setTurretAngleDegrees(double targetAngleDegrees) {
        requestedAngleDegrees = normalizeSignedAngleDegrees(targetAngleDegrees);
        requestedUnwrappedAngleDegrees =
            chooseFrontSafeUnwrappedTarget(getTurretUnwrappedAngleDegrees(), requestedAngleDegrees);
        turretMotor.setControl(
            positionRequest.withPosition(degreesToRotorRotations(requestedUnwrappedAngleDegrees)));
    }

    public Command setTurretAngleCommand(double targetAngleDegrees) {
        return runOnce(() -> setTurretAngleDegrees(targetAngleDegrees));
    }

    public Command frontCommand() {
        return setTurretAngleCommand(0.0);
    }

    public Command backCommand() {
        return setTurretAngleCommand(180.0);
    }

    public Command clockwise90Command() {
        return setTurretAngleCommand(90.0);
    }

    public Command counterClockwise90Command() {
        return setTurretAngleCommand(-90.0);
    }

    public Command rearPresetCommand() {
        return setTurretAngleCommand(180.0);
    }

    public Command leftPresetCommand() {
        return setTurretAngleCommand(-90.0);
    }

    public Command rearLeftCornerPresetCommand() {
        return setTurretAngleCommand(-135.0);
    }

    public Command rearRightCornerPresetCommand() {
        return setTurretAngleCommand(135.0);
    }

    public double calculateHubAimAngleDegrees() {
        return calculateAimAngleDegrees(getCurrentHubTranslation());
    }

    public double calculateBallTimeOfFlightSeconds(double distanceMeters) {
        return ShooterConstants.ballTimeOfFlightSecondsForDistanceMeters(distanceMeters);
    }

    public boolean isInCenterField() {
        double robotX = drivetrain.getEstimatedPose().getX();
        return robotX >= TurretConstants.kCenterFieldXMinMeters
            && robotX <= TurretConstants.kCenterFieldXMaxMeters;
    }

    public Translation2d getCurrentAlignTargetTranslation() {
        if (isInCenterField()) {
            return getCurrentPassCornerTranslation();
        }

        return getCurrentHubTranslation();
    }

    public Translation2d getCurrentPassCornerTranslation() {
        return DriverStation.getAlliance().orElse(Alliance.Blue) == Alliance.Red
            ? FieldConstants.kRedPassCornerPosition
            : FieldConstants.kBluePassCornerPosition;
    }

    public String getAlignTargetMode() {
        return isInCenterField() ? "Pass" : "Hub";
    }

    public double calculateAlignAngleDegrees() {
        return calculateAimAngleDegrees(getCurrentAlignTargetTranslation());
    }

    public double calculateCompensatedAlignAngleDegrees() {
        return calculateCompensatedAimAngleDegrees(getCurrentAlignTargetTranslation());
    }

    private double calculateAimAngleDegrees(Translation2d targetTranslation) {
        Pose2d robotPose = drivetrain.getEstimatedPose();
        Translation2d robotTranslation = robotPose.getTranslation();

        double fieldAngleDegrees = Math.toDegrees(
            Math.atan2(
                targetTranslation.getY() - robotTranslation.getY(),
                targetTranslation.getX() - robotTranslation.getX()));
        double robotRelativeCounterClockwiseDegrees =
            normalizeSignedAngleDegrees(fieldAngleDegrees - robotPose.getRotation().getDegrees());

        // Turret commands use clockwise-positive angles, opposite WPILib's
        // counterclockwise-positive rotation convention.
        return normalizeSignedAngleDegrees(-robotRelativeCounterClockwiseDegrees);
    }

    public double calculateCompensatedHubAimAngleDegrees() {
        return calculateCompensatedAimAngleDegrees(getCurrentHubTranslation());
    }

    public double calculateCompensatedPassCornerAimAngleDegrees() {
        return calculateCompensatedAimAngleDegrees(getCurrentPassCornerTranslation());
    }

    private double calculateCompensatedAimAngleDegrees(Translation2d targetTranslation) {
        Pose2d robotPose = drivetrain.getEstimatedPose();
        Translation2d robotTranslation = robotPose.getTranslation();
        Translation2d robotToTarget = targetTranslation.minus(robotTranslation);
        double distanceToTargetMeters = robotToTarget.getNorm();

        if (distanceToTargetMeters < 1e-6) {
            return calculateAimAngleDegrees(targetTranslation);
        }

        var chassisSpeeds = drivetrain.getState().Speeds;
        Translation2d fieldRelativeVelocity = new Translation2d(
            chassisSpeeds.vxMetersPerSecond,
            chassisSpeeds.vyMetersPerSecond).rotateBy(robotPose.getRotation());

        double lookaheadSeconds = calculateBallTimeOfFlightSeconds(distanceToTargetMeters);

        Translation2d shotDirectionUnit = robotToTarget.div(distanceToTargetMeters);
        Translation2d lateralDirectionUnit = new Translation2d(
            -shotDirectionUnit.getY(),
            shotDirectionUnit.getX());

        double lateralVelocityMetersPerSecond =
            fieldRelativeVelocity.getX() * lateralDirectionUnit.getX()
                + fieldRelativeVelocity.getY() * lateralDirectionUnit.getY();
        double translationalLeadDegrees = Math.toDegrees(
            Math.atan2(
                lateralVelocityMetersPerSecond
                    * lookaheadSeconds
                    * TurretConstants.kTurretTranslationalLeadGain,
                distanceToTargetMeters));

        double rotationalLeadDegrees =
            Math.toDegrees(chassisSpeeds.omegaRadiansPerSecond)
                * lookaheadSeconds
                * TurretConstants.kTurretRotationalLeadGain;

        return normalizeSignedAngleDegrees(
            calculateAimAngleDegrees(targetTranslation)
                - translationalLeadDegrees
                - rotationalLeadDegrees);
    }

    public Command aimAtHubCommand() {
        return run(() -> setTurretAngleDegrees(calculateHubAimAngleDegrees()));
    }

    public Command aimAtHubOnceCommand() {
        return runOnce(() -> setTurretAngleDegrees(calculateHubAimAngleDegrees()));
    }

    public Command aimAtHubWithMotionCompCommand() {
        return run(() -> setTurretAngleDegrees(calculateCompensatedHubAimAngleDegrees()));
    }

    public Command aimAtHubWithMotionCompOnceCommand() {
        return runOnce(() -> setTurretAngleDegrees(calculateCompensatedHubAimAngleDegrees()));
    }

    public Command aimAtPassCornerWithMotionCompCommand() {
        return run(() -> setTurretAngleDegrees(calculateCompensatedPassCornerAimAngleDegrees()));
    }

    public Command alignCommand() {
        return run(() -> setTurretAngleDegrees(calculateCompensatedAlignAngleDegrees()));
    }

    public Command alignOnceCommand() {
        return runOnce(() -> setTurretAngleDegrees(calculateCompensatedAlignAngleDegrees()));
    }

    private void seedStartAngle() {
        turretMotor.setPosition(
            degreesToRotorRotations(TurretConstants.kTurretStartAngleDegrees));
    }

    private Translation2d getCurrentHubTranslation() {
        return DriverStation.getAlliance().orElse(Alliance.Blue) == Alliance.Red
            ? FieldConstants.kRedTowerPosition
            : FieldConstants.kBlueTowerPosition;
    }

    private void configureMotor() {
        var motorOutputConfigs = new MotorOutputConfigs();
        motorOutputConfigs.NeutralMode = NeutralModeValue.Brake;

        var currentLimitConfigs = new CurrentLimitsConfigs();
        currentLimitConfigs.StatorCurrentLimit = TurretConstants.kTurretCurrentLimit;
        currentLimitConfigs.StatorCurrentLimitEnable = true;

        var slot0Configs = new Slot0Configs();
        slot0Configs.kP = TurretConstants.kTurretKP;
        slot0Configs.kI = TurretConstants.kTurretKI;
        slot0Configs.kD = TurretConstants.kTurretKD;
        slot0Configs.kG = TurretConstants.kTurretKG;

        var motionMagicConfigs = new MotionMagicConfigs();
        motionMagicConfigs.MotionMagicCruiseVelocity =
            TurretConstants.kTurretCruiseVelocityRotationsPerSecond;
        motionMagicConfigs.MotionMagicAcceleration =
            TurretConstants.kTurretAccelerationRotationsPerSecondSquared;
        motionMagicConfigs.MotionMagicJerk =
            TurretConstants.kTurretJerkRotationsPerSecondCubed;

        turretMotor.getConfigurator().apply(motorOutputConfigs);
        turretMotor.getConfigurator().apply(currentLimitConfigs);
        turretMotor.getConfigurator().apply(slot0Configs);
        turretMotor.getConfigurator().apply(motionMagicConfigs);
    }

    private double chooseFrontSafeUnwrappedTarget(double currentUnwrappedAngleDegrees,
            double targetWrappedAngleDegrees) {
        double bestTarget = targetWrappedAngleDegrees;
        double bestDelta = Double.POSITIVE_INFINITY;

        for (int wrapOffset = -2; wrapOffset <= 2; wrapOffset++) {
            double candidateTarget = targetWrappedAngleDegrees + (wrapOffset * 360.0);
            double candidateDelta = Math.abs(candidateTarget - currentUnwrappedAngleDegrees);

            if (!crossesFrontSeam(currentUnwrappedAngleDegrees, candidateTarget)
                    && candidateDelta < bestDelta) {
                bestTarget = candidateTarget;
                bestDelta = candidateDelta;
            }
        }

        if (bestDelta < Double.POSITIVE_INFINITY) {
            return bestTarget;
        }

        return targetWrappedAngleDegrees;
    }

    private boolean crossesFrontSeam(double startAngleDegrees, double endAngleDegrees) {
        if (Math.abs(startAngleDegrees - endAngleDegrees) < 1e-9) {
            return false;
        }

        double minAngle = Math.min(startAngleDegrees, endAngleDegrees);
        double maxAngle = Math.max(startAngleDegrees, endAngleDegrees);
        int firstSeamIndex = (int) Math.ceil(minAngle / 360.0);
        int lastSeamIndex = (int) Math.floor(maxAngle / 360.0);

        for (int seamIndex = firstSeamIndex; seamIndex <= lastSeamIndex; seamIndex++) {
            double seamAngle = seamIndex * 360.0;
            if (seamAngle > minAngle && seamAngle < maxAngle) {
                return true;
            }
        }

        return false;
    }

    private double normalizeSignedAngleDegrees(double angleDegrees) {
        double normalizedAngle = angleDegrees % 360.0;
        if (normalizedAngle > 180.0) {
            normalizedAngle -= 360.0;
        }
        if (normalizedAngle <= -180.0) {
            normalizedAngle += 360.0;
        }
        return normalizedAngle;
    }

    private double degreesToRotorRotations(double angleDegrees) {
        return angleDegrees / 360.0 * TurretConstants.kTurretGearRatio;
    }

    private double rotorRotationsToDegrees(double rotorRotations) {
        return rotorRotations / TurretConstants.kTurretGearRatio * 360.0;
    }
}
