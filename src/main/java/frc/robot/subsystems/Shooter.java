package frc.robot.subsystems;

import com.ctre.phoenix6.configs.CurrentLimitsConfigs;
import com.ctre.phoenix6.configs.FeedbackConfigs;
import com.ctre.phoenix6.configs.MotionMagicConfigs;
import com.ctre.phoenix6.configs.MotorOutputConfigs;
import com.ctre.phoenix6.configs.Slot0Configs;
import com.ctre.phoenix6.controls.Follower;
import com.ctre.phoenix6.controls.MotionMagicVelocityVoltage;
import com.ctre.phoenix6.controls.VoltageOut;
import com.ctre.phoenix6.hardware.TalonFX;
import com.ctre.phoenix6.signals.MotorAlignmentValue;
import com.ctre.phoenix6.signals.NeutralModeValue;

import edu.wpi.first.math.MathUtil;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.networktables.NetworkTableEntry;
import edu.wpi.first.networktables.NetworkTableInstance;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.DriverStation.Alliance;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants.FieldConstants;
import frc.robot.Constants.ShooterConstants;
import java.util.function.DoubleUnaryOperator;

public class Shooter extends SubsystemBase {
    private static final String kElasticShooterVelocityOffsetTopicName =
        "LerpVelocityMagnitudeOffsetRPS";
    private static final String kElasticTrackedShotVelocitySliderTopicName =
        "TrackedShotVelocitySliderRPS";

    private final TalonFX shooterLeader = new TalonFX(ShooterConstants.kShooterMotor1ID);
    private final TalonFX shooterFollower = new TalonFX(ShooterConstants.kShooterMotor2ID);
    private final Drivetrain drivetrain;
    private final NetworkTableEntry shooterLerpVelocityMagnitudeOffsetEntry =
        NetworkTableInstance.getDefault()
            .getTable("Elastic")
            .getSubTable("Shooter")
            .getEntry(kElasticShooterVelocityOffsetTopicName);
    private final NetworkTableEntry trackedShotVelocitySliderEntry =
        NetworkTableInstance.getDefault()
            .getTable("Elastic")
            .getSubTable("Shooter")
            .getEntry(kElasticTrackedShotVelocitySliderTopicName);

    private final MotionMagicVelocityVoltage velocityRequest = new MotionMagicVelocityVoltage(0);
    private final VoltageOut stopRequest = new VoltageOut(0);
    private final Follower followerRequest =
        new Follower(shooterLeader.getDeviceID(), MotorAlignmentValue.Opposed);

    private double targetVelocityRotationsPerSecond =
        ShooterConstants.kShooterTargetVelocityRotationsPerSecond;

    public Shooter(Drivetrain drivetrain) {
        this.drivetrain = drivetrain;
        shooterLerpVelocityMagnitudeOffsetEntry.setDefaultDouble(0.0);
        trackedShotVelocitySliderEntry.setDefaultDouble(
            Math.abs(ShooterConstants.kShooterTargetVelocityRotationsPerSecond));

        configureMotor(shooterLeader);
        configureMotor(shooterFollower);
        shooterFollower.setControl(followerRequest);
        stop();
    }

    public double getDistanceToHubMeters() {
        Translation2d robotTranslation = drivetrain.getEstimatedPose().getTranslation();
        return robotTranslation.getDistance(getCurrentHubTranslation());
    }

    public double getDistanceToTargetMeters(Translation2d targetTranslation) {
        Translation2d robotTranslation = drivetrain.getEstimatedPose().getTranslation();
        return robotTranslation.getDistance(targetTranslation);
    }

    public double getTargetVelocityRotationsPerSecond() {
        return targetVelocityRotationsPerSecond;
    }

    public double getMeasuredVelocityRotationsPerSecond() {
        return shooterLeader.getVelocity().getValueAsDouble();
    }

    public boolean isNearTargetVelocity(double toleranceRotationsPerSecond) {
        return Math.abs(getMeasuredVelocityRotationsPerSecond() - targetVelocityRotationsPerSecond)
            <= toleranceRotationsPerSecond;
    }

    public boolean isNearTargetVelocityMagnitude(double toleranceRotationsPerSecond) {
        return Math.abs(
            Math.abs(getMeasuredVelocityRotationsPerSecond())
                - Math.abs(targetVelocityRotationsPerSecond))
            <= toleranceRotationsPerSecond;
    }

    public boolean isAtOrAboveTargetVelocityMagnitude(double toleranceRotationsPerSecond) {
        return Math.abs(getMeasuredVelocityRotationsPerSecond())
            >= Math.abs(targetVelocityRotationsPerSecond) - toleranceRotationsPerSecond;
    }

    public void setVelocityRotationsPerSecond(double targetVelocityRotationsPerSecond) {
        double targetVelocityMagnitudeRotationsPerSecond = Math.min(
            Math.abs(targetVelocityRotationsPerSecond),
            ShooterConstants.kShooterCommandMaxVelocityRotationsPerSecond);
        this.targetVelocityRotationsPerSecond = Math.copySign(
            targetVelocityMagnitudeRotationsPerSecond,
            targetVelocityRotationsPerSecond);
        shooterLeader.setControl(
            velocityRequest.withVelocity(this.targetVelocityRotationsPerSecond));
        shooterFollower.setControl(followerRequest);
    }

    public void updateVelocityForCurrentDistance() {
        setVelocityRotationsPerSecond(
            getRequiredVelocityForDistanceMeters(getCompensatedDistanceToHubMeters()));
    }

    public void updateBackspinBallisticVelocityForCurrentDistance() {
        setVelocityRotationsPerSecond(
            getRequiredBackspinBallisticVelocityForDistanceMeters(
                getCompensatedDistanceToHubMeters(
                    this::getRequiredBackspinBallisticVelocityForDistanceMeters)));
    }

    public void updateVelocityForTarget(Translation2d targetTranslation) {
        setVelocityRotationsPerSecond(getRequiredVelocityForTarget(targetTranslation));
    }

    public double getRequiredVelocityForTarget(Translation2d targetTranslation) {
        return getRequiredVelocityForDistanceMeters(
            getCompensatedDistanceToTargetMeters(targetTranslation));
    }

    public void stop() {
        shooterLeader.setControl(stopRequest.withOutput(0));
        shooterFollower.setControl(followerRequest);
    }

    public Command setVelocityCommand(double targetVelocityRotationsPerSecond) {
        return runOnce(() -> setVelocityRotationsPerSecond(targetVelocityRotationsPerSecond));
    }

    public Command runDistanceBasedVelocityCommand() {
        return runEnd(this::updateVelocityForCurrentDistance, this::stop);
    }

    public Command runBackspinBallisticVelocityCommand() {
        return runEnd(this::updateBackspinBallisticVelocityForCurrentDistance, this::stop);
    }

    public void updateVelocityFromTrackedShotSlider() {
        setVelocityRotationsPerSecond(-getTrackedShotSliderVelocityMagnitudeRotationsPerSecond());
    }

    public Command runTrackedShotSliderVelocityCommand() {
        return runEnd(this::updateVelocityFromTrackedShotSlider, this::stop);
    }

    public Command updateDistanceBasedVelocityOnceCommand() {
        return runOnce(this::updateVelocityForCurrentDistance);
    }

    public Command updateBackspinBallisticVelocityOnceCommand() {
        return runOnce(this::updateBackspinBallisticVelocityForCurrentDistance);
    }

    public Command stopCommand() {
        return runOnce(this::stop);
    }

    private Translation2d getCurrentHubTranslation() {
        return DriverStation.getAlliance().orElse(Alliance.Blue) == Alliance.Red
            ? FieldConstants.kRedHubPosition
            : FieldConstants.kBlueHubPosition;
    }

    private double getCompensatedDistanceToHubMeters() {
        return getCompensatedDistanceToHubMeters(this::getRequiredVelocityForDistanceMeters);
    }

    private double getCompensatedDistanceToHubMeters(
            DoubleUnaryOperator requiredVelocityForDistanceMetersSupplier) {
        return getCompensatedDistanceToTargetMeters(
            getCurrentHubTranslation(),
            requiredVelocityForDistanceMetersSupplier);
    }

    private double getCompensatedDistanceToTargetMeters(Translation2d targetTranslation) {
        return getCompensatedDistanceToTargetMeters(
            targetTranslation,
            this::getRequiredVelocityForDistanceMeters);
    }

    private double getCompensatedDistanceToTargetMeters(
            Translation2d targetTranslation,
            DoubleUnaryOperator requiredVelocityForDistanceMetersSupplier) {
        Pose2d robotPose = drivetrain.getEstimatedPose();
        Translation2d robotTranslation = robotPose.getTranslation();
        var chassisSpeeds = drivetrain.getState().Speeds;
        Translation2d fieldRelativeVelocity = new Translation2d(
            chassisSpeeds.vxMetersPerSecond,
            chassisSpeeds.vyMetersPerSecond).rotateBy(robotPose.getRotation());

        double currentDistanceMeters = robotTranslation.getDistance(targetTranslation);
        double lookaheadSeconds = calculateBallTimeOfFlightSeconds(
            currentDistanceMeters,
            requiredVelocityForDistanceMetersSupplier.applyAsDouble(currentDistanceMeters));

        // Recompute once from the predicted pose so the setpoint is based on where
        // the robot will be when the note reaches the target.
        Translation2d predictedTranslation =
            robotTranslation.plus(fieldRelativeVelocity.times(lookaheadSeconds));
        double predictedDistanceMeters = predictedTranslation.getDistance(targetTranslation);

        return predictedTranslation.plus(
            fieldRelativeVelocity.times(calculateBallTimeOfFlightSeconds(
                predictedDistanceMeters,
                requiredVelocityForDistanceMetersSupplier.applyAsDouble(predictedDistanceMeters))))
            .getDistance(targetTranslation);
    }

    private double getRequiredVelocityForDistanceMeters(double distanceMeters) {
        return applyDistanceShotVelocityOffset(
            ShooterConstants.velocityForDistanceMeters(distanceMeters));
    }

    private double getRequiredBackspinBallisticVelocityForDistanceMeters(double distanceMeters) {
        return applyDistanceShotVelocityOffset(
            ShooterConstants.backspinBallisticVelocityForDistanceMeters(distanceMeters));
    }

    private double applyDistanceShotVelocityOffset(double baseVelocityRotationsPerSecond) {
        double tunedVelocityMagnitudeRotationsPerSecond = Math.max(
            0.0,
            Math.abs(baseVelocityRotationsPerSecond)
                + ShooterConstants.kDistanceShotBaseVelocityTrimRotationsPerSecond
                + getLerpVelocityMagnitudeOffsetRotationsPerSecond());
        return Math.copySign(
            tunedVelocityMagnitudeRotationsPerSecond,
            baseVelocityRotationsPerSecond);
    }

    private double calculateBallTimeOfFlightSeconds(
            double distanceMeters,
            double shooterVelocityRotationsPerSecond) {
        return ShooterConstants.ballTimeOfFlightSecondsForDistanceMeters(
            distanceMeters,
            Math.abs(shooterVelocityRotationsPerSecond));
    }

    public double getLerpVelocityMagnitudeOffsetRotationsPerSecond() {
        return shooterLerpVelocityMagnitudeOffsetEntry.getDouble(0.0);
    }

    public double getTrackedShotSliderVelocityMagnitudeRotationsPerSecond() {
        return MathUtil.clamp(
            Math.abs(trackedShotVelocitySliderEntry.getDouble(
                Math.abs(ShooterConstants.kShooterTargetVelocityRotationsPerSecond))),
            0.0,
            ShooterConstants.kShooterCommandMaxVelocityRotationsPerSecond);
    }

    private void configureMotor(TalonFX motor) {
        var motorOutputConfigs = new MotorOutputConfigs();
        motorOutputConfigs.NeutralMode = NeutralModeValue.Coast;

        var currentLimitConfigs = new CurrentLimitsConfigs();
        currentLimitConfigs.StatorCurrentLimit = ShooterConstants.kShooterCurrentLimit;
        currentLimitConfigs.StatorCurrentLimitEnable = true;

        var feedbackConfigs = new FeedbackConfigs();
        feedbackConfigs.SensorToMechanismRatio = ShooterConstants.kShooterSensorToMechanismRatio;

        var slot0Configs = new Slot0Configs();
        slot0Configs.kS = ShooterConstants.kShooterVelocityKS;
        slot0Configs.kV = ShooterConstants.kShooterVelocityKV;
        slot0Configs.kA = ShooterConstants.kShooterVelocityKA;
        slot0Configs.kP = ShooterConstants.kShooterVelocityKP;
        slot0Configs.kI = ShooterConstants.kShooterVelocityKI;
        slot0Configs.kD = ShooterConstants.kShooterVelocityKD;

        var motionMagicConfigs = new MotionMagicConfigs();
        motionMagicConfigs.MotionMagicAcceleration =
            ShooterConstants.kShooterAccelerationRotationsPerSecondSquared;
        motionMagicConfigs.MotionMagicJerk =
            ShooterConstants.kShooterJerkRotationsPerSecondCubed;

        motor.getConfigurator().apply(motorOutputConfigs);
        motor.getConfigurator().apply(currentLimitConfigs);
        motor.getConfigurator().apply(feedbackConfigs);
        motor.getConfigurator().apply(slot0Configs);
        motor.getConfigurator().apply(motionMagicConfigs);
    }
}
