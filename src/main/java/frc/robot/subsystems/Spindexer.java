package frc.robot.subsystems;

import com.ctre.phoenix6.configs.CurrentLimitsConfigs;
import com.ctre.phoenix6.configs.MotionMagicConfigs;
import com.ctre.phoenix6.configs.MotorOutputConfigs;
import com.ctre.phoenix6.configs.Slot0Configs;
import com.ctre.phoenix6.controls.MotionMagicVelocityVoltage;
import com.ctre.phoenix6.controls.VoltageOut;
import com.ctre.phoenix6.hardware.TalonFX;
import com.ctre.phoenix6.signals.NeutralModeValue;

import edu.wpi.first.networktables.NetworkTableEntry;
import edu.wpi.first.networktables.NetworkTableInstance;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants.SpindexerConstants;

public class Spindexer extends SubsystemBase {
    private static final String kElasticTrackedShotVelocityMultiplierTopicName =
        "TrackedShotVelocityMultiplier";

    private final TalonFX spindexerMotor = new TalonFX(SpindexerConstants.kSpindexerMotorID);
    private final TalonFX receiverMotor = new TalonFX(SpindexerConstants.kReceiveMotorID);
    private final NetworkTableEntry trackedShotVelocityMultiplierEntry =
        NetworkTableInstance.getDefault()
            .getTable("Elastic")
            .getSubTable("Spindexer")
            .getEntry(kElasticTrackedShotVelocityMultiplierTopicName);

    private final MotionMagicVelocityVoltage spindexerVelocityRequest =
        new MotionMagicVelocityVoltage(0);
    private final MotionMagicVelocityVoltage receiverVelocityRequest =
        new MotionMagicVelocityVoltage(0);
    private final VoltageOut stopRequest = new VoltageOut(0);
    private double spindexerTargetVelocityRotationsPerSecond = 0.0;
    private double receiverTargetVelocityRotationsPerSecond = 0.0;

    public Spindexer() {
        trackedShotVelocityMultiplierEntry.setDefaultDouble(1.0);
        configureMotor(spindexerMotor, SpindexerConstants.kSpindexerCurrentLimit);
        configureMotor(receiverMotor, SpindexerConstants.kReceiverCurrentLimit);
        stop();
    }

    public void feedForward() {
        setFeedVelocityRotationsPerSecond(SpindexerConstants.kSpindexerVelocityRotationsPerSecond);
    }

    public void feedReverse() {
        setFeedVelocityRotationsPerSecond(-SpindexerConstants.kSpindexerVelocityRotationsPerSecond);
    }

    public void feedForShooterVelocityRotationsPerSecond(double shooterVelocityRotationsPerSecond) {
        setFeedVelocityRotationsPerSecond(
            SpindexerConstants.spindexerVelocityForShooterVelocityRotationsPerSecond(
                shooterVelocityRotationsPerSecond));
    }

    public void feedForShooterVelocityRotationsPerSecondWithTrackedShotMultiplier(
            double shooterVelocityRotationsPerSecond) {
        setFeedVelocityRotationsPerSecond(
            SpindexerConstants.spindexerVelocityForShooterVelocityRotationsPerSecond(
                shooterVelocityRotationsPerSecond * getTrackedShotVelocityMultiplier()));
    }

    public void setFeedVelocityRotationsPerSecond(double spindexerVelocityRotationsPerSecond) {
        spindexerTargetVelocityRotationsPerSecond = spindexerVelocityRotationsPerSecond;
        receiverTargetVelocityRotationsPerSecond =
            SpindexerConstants.receiverVelocityForSpindexerVelocityRotationsPerSecond(
                spindexerVelocityRotationsPerSecond);
        applyVelocityTargets(
            spindexerTargetVelocityRotationsPerSecond,
            receiverTargetVelocityRotationsPerSecond);
    }

    public void stop() {
        spindexerTargetVelocityRotationsPerSecond = 0.0;
        receiverTargetVelocityRotationsPerSecond = 0.0;
        spindexerMotor.setControl(stopRequest.withOutput(0));
        receiverMotor.setControl(stopRequest.withOutput(0));
    }

    public double getSpindexerTargetVelocityRotationsPerSecond() {
        return spindexerTargetVelocityRotationsPerSecond;
    }

    public double getReceiverTargetVelocityRotationsPerSecond() {
        return receiverTargetVelocityRotationsPerSecond;
    }

    public double getSpindexerMeasuredVelocityRotationsPerSecond() {
        return spindexerMotor.getVelocity().getValueAsDouble();
    }

    public double getReceiverMeasuredVelocityRotationsPerSecond() {
        return receiverMotor.getVelocity().getValueAsDouble();
    }

    public double getSpindexerCurrentAmps() {
        return spindexerMotor.getStatorCurrent().getValueAsDouble();
    }

    public double getReceiverCurrentAmps() {
        return receiverMotor.getStatorCurrent().getValueAsDouble();
    }

    public double getTrackedShotVelocityMultiplier() {
        return Math.max(0.0, trackedShotVelocityMultiplierEntry.getDouble(1.0));
    }

    public Command feedForwardCommand() {
        return runOnce(this::feedForward);
    }

    public Command feedReverseCommand() {
        return runOnce(this::feedReverse);
    }

    public Command setFeedVelocityCommand(double spindexerVelocityRotationsPerSecond) {
        return runOnce(() -> setFeedVelocityRotationsPerSecond(spindexerVelocityRotationsPerSecond));
    }

    public Command stopCommand() {
        return runOnce(this::stop);
    }

    @Override
    public void periodic() {
        applyVelocityTargets(
            spindexerTargetVelocityRotationsPerSecond,
            receiverTargetVelocityRotationsPerSecond);
    }

    private void configureMotor(TalonFX motor, double statorCurrentLimit) {
        var motorOutputConfigs = new MotorOutputConfigs();
        motorOutputConfigs.NeutralMode = NeutralModeValue.Brake;

        var currentLimitConfigs = new CurrentLimitsConfigs();
        currentLimitConfigs.StatorCurrentLimit = statorCurrentLimit;
        currentLimitConfigs.StatorCurrentLimitEnable = true;

        var slot0Configs = new Slot0Configs();
        slot0Configs.kP = SpindexerConstants.kFeedVelocityKP;
        slot0Configs.kI = SpindexerConstants.kFeedVelocityKI;
        slot0Configs.kD = SpindexerConstants.kFeedVelocityKD;
        slot0Configs.kS = SpindexerConstants.kFeedVelocityKS;
        slot0Configs.kV = SpindexerConstants.kFeedVelocityKV;
        slot0Configs.kA = SpindexerConstants.kFeedVelocityKA;

        var motionMagicConfigs = new MotionMagicConfigs();
        motionMagicConfigs.MotionMagicAcceleration =
            SpindexerConstants.kFeedAccelerationRotationsPerSecondSquared;
        motionMagicConfigs.MotionMagicJerk =
            SpindexerConstants.kFeedJerkRotationsPerSecondCubed;

        motor.getConfigurator().apply(motorOutputConfigs);
        motor.getConfigurator().apply(currentLimitConfigs);
        motor.getConfigurator().apply(slot0Configs);
        motor.getConfigurator().apply(motionMagicConfigs);
    }

    private void applyVelocityTargets(
            double spindexerVelocityRotationsPerSecond,
            double receiverVelocityRotationsPerSecond) {
        spindexerMotor.setControl(
            spindexerVelocityRequest.withVelocity(spindexerVelocityRotationsPerSecond));
        receiverMotor.setControl(
            receiverVelocityRequest.withVelocity(receiverVelocityRotationsPerSecond));
    }
}
