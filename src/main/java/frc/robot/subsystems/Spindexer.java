package frc.robot.subsystems;

import com.ctre.phoenix6.configs.CurrentLimitsConfigs;
import com.ctre.phoenix6.configs.MotionMagicConfigs;
import com.ctre.phoenix6.configs.MotorOutputConfigs;
import com.ctre.phoenix6.configs.Slot0Configs;
import com.ctre.phoenix6.controls.MotionMagicVelocityVoltage;
import com.ctre.phoenix6.controls.VoltageOut;
import com.ctre.phoenix6.hardware.TalonFX;
import com.ctre.phoenix6.signals.NeutralModeValue;

import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants.SpindexerConstants;

public class Spindexer extends SubsystemBase {
    private final TalonFX spindexerMotor = new TalonFX(SpindexerConstants.kSpindexerMotorID);
    private final TalonFX receiverMotor = new TalonFX(SpindexerConstants.kReceiveMotorID);

    private final MotionMagicVelocityVoltage spindexerVelocityRequest =
        new MotionMagicVelocityVoltage(0);
    private final MotionMagicVelocityVoltage receiverVelocityRequest =
        new MotionMagicVelocityVoltage(0);
    private final VoltageOut stopRequest = new VoltageOut(0);
    private double spindexerTargetVelocityRotationsPerSecond = 0.0;
    private double receiverTargetVelocityRotationsPerSecond = 0.0;
    private boolean jamRecoveryActive = false;
    private double jamRecoveryEndTimeSeconds = Double.NEGATIVE_INFINITY;
    private double jamConditionStartTimeSeconds = Double.NEGATIVE_INFINITY;
    private double commandStartTimeSeconds = Double.NEGATIVE_INFINITY;
    private double jamRecoveryCooldownEndTimeSeconds = Double.NEGATIVE_INFINITY;

    public Spindexer() {
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

    public void setFeedVelocityRotationsPerSecond(double spindexerVelocityRotationsPerSecond) {
        boolean wasStopped = Math.abs(spindexerTargetVelocityRotationsPerSecond) < 1e-6;
        spindexerTargetVelocityRotationsPerSecond = spindexerVelocityRotationsPerSecond;
        receiverTargetVelocityRotationsPerSecond =
            spindexerVelocityRotationsPerSecond
                * (SpindexerConstants.kReceiverSpeedRatioToShooter
                    / SpindexerConstants.kSpindexerSpeedRatioToShooter);
        if (wasStopped && Math.abs(spindexerVelocityRotationsPerSecond) >= 1e-6) {
            commandStartTimeSeconds = Timer.getFPGATimestamp();
        }

        if (!jamRecoveryActive) {
            applyVelocityTargets(
                spindexerTargetVelocityRotationsPerSecond,
                receiverTargetVelocityRotationsPerSecond);
        }
    }

    public void stop() {
        spindexerTargetVelocityRotationsPerSecond = 0.0;
        receiverTargetVelocityRotationsPerSecond = 0.0;
        jamRecoveryActive = false;
        jamRecoveryEndTimeSeconds = Double.NEGATIVE_INFINITY;
        jamConditionStartTimeSeconds = Double.NEGATIVE_INFINITY;
        commandStartTimeSeconds = Double.NEGATIVE_INFINITY;
        jamRecoveryCooldownEndTimeSeconds = Double.NEGATIVE_INFINITY;
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

    public boolean isJamRecoveryActive() {
        return jamRecoveryActive;
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
        if (Math.abs(spindexerTargetVelocityRotationsPerSecond) < 1e-6) {
            jamConditionStartTimeSeconds = Double.NEGATIVE_INFINITY;
            return;
        }

        double nowSeconds = Timer.getFPGATimestamp();
        if (jamRecoveryActive) {
            if (nowSeconds >= jamRecoveryEndTimeSeconds) {
                jamRecoveryActive = false;
                jamRecoveryCooldownEndTimeSeconds =
                    nowSeconds + SpindexerConstants.kJamRecoveryCooldownSeconds;
                jamConditionStartTimeSeconds = Double.NEGATIVE_INFINITY;
                applyVelocityTargets(
                    spindexerTargetVelocityRotationsPerSecond,
                    receiverTargetVelocityRotationsPerSecond);
            } else {
                applyVelocityTargets(
                    -spindexerTargetVelocityRotationsPerSecond
                        * SpindexerConstants.kJamRecoveryReverseScale,
                    -receiverTargetVelocityRotationsPerSecond
                        * SpindexerConstants.kJamRecoveryReverseScale);
            }
            return;
        }

        if (shouldStartJamRecovery(nowSeconds)) {
            jamRecoveryActive = true;
            jamRecoveryEndTimeSeconds = nowSeconds + SpindexerConstants.kJamRecoverySeconds;
            applyVelocityTargets(
                -spindexerTargetVelocityRotationsPerSecond
                    * SpindexerConstants.kJamRecoveryReverseScale,
                -receiverTargetVelocityRotationsPerSecond
                    * SpindexerConstants.kJamRecoveryReverseScale);
            return;
        }

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

    private boolean shouldStartJamRecovery(double nowSeconds) {
        double commandedSpeedMagnitude = Math.abs(spindexerTargetVelocityRotationsPerSecond);
        if (commandedSpeedMagnitude < 1e-6) {
            jamConditionStartTimeSeconds = Double.NEGATIVE_INFINITY;
            return false;
        }

        if (commandStartTimeSeconds > 0.0
                && nowSeconds - commandStartTimeSeconds < SpindexerConstants.kJamDetectionDelaySeconds) {
            jamConditionStartTimeSeconds = Double.NEGATIVE_INFINITY;
            return false;
        }

        if (nowSeconds < jamRecoveryCooldownEndTimeSeconds) {
            jamConditionStartTimeSeconds = Double.NEGATIVE_INFINITY;
            return false;
        }

        double measuredSpindexerSpeedMagnitude = Math.abs(getSpindexerMeasuredVelocityRotationsPerSecond());
        double measuredReceiverSpeedMagnitude = Math.abs(getReceiverMeasuredVelocityRotationsPerSecond());
        double minimumHealthySpeed =
            commandedSpeedMagnitude * SpindexerConstants.kJamDetectionVelocityRatio;

        boolean lowVelocity =
            measuredSpindexerSpeedMagnitude < minimumHealthySpeed
                || measuredReceiverSpeedMagnitude < minimumHealthySpeed;
        boolean highCurrent =
            getSpindexerCurrentAmps() >= SpindexerConstants.kJamCurrentThresholdAmps
                || getReceiverCurrentAmps() >= SpindexerConstants.kJamCurrentThresholdAmps;

        if (!(lowVelocity && highCurrent)) {
            jamConditionStartTimeSeconds = Double.NEGATIVE_INFINITY;
            return false;
        }

        if (jamConditionStartTimeSeconds == Double.NEGATIVE_INFINITY) {
            jamConditionStartTimeSeconds = nowSeconds;
            return false;
        }

        return nowSeconds - jamConditionStartTimeSeconds
            >= SpindexerConstants.kJamDetectionConfirmSeconds;
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
