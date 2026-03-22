package frc.robot.subsystems;

import com.ctre.phoenix6.configs.CurrentLimitsConfigs;
import com.ctre.phoenix6.configs.MotionMagicConfigs;
import com.ctre.phoenix6.configs.MotorOutputConfigs;
import com.ctre.phoenix6.configs.Slot0Configs;
import com.ctre.phoenix6.controls.MotionMagicVelocityVoltage;
import com.ctre.phoenix6.controls.VoltageOut;
import com.ctre.phoenix6.hardware.TalonFX;
import com.ctre.phoenix6.signals.NeutralModeValue;

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
        spindexerTargetVelocityRotationsPerSecond = spindexerVelocityRotationsPerSecond;
        receiverTargetVelocityRotationsPerSecond =
            spindexerVelocityRotationsPerSecond
                * (SpindexerConstants.kReceiverSpeedRatioToShooter
                    / SpindexerConstants.kSpindexerSpeedRatioToShooter);
        spindexerMotor.setControl(
            spindexerVelocityRequest.withVelocity(spindexerTargetVelocityRotationsPerSecond));
        receiverMotor.setControl(
            receiverVelocityRequest.withVelocity(receiverTargetVelocityRotationsPerSecond));
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
}
