package frc.robot.subsystems;

import com.ctre.phoenix6.configs.CurrentLimitsConfigs;
import com.ctre.phoenix6.configs.MotionMagicConfigs;
import com.ctre.phoenix6.configs.MotorOutputConfigs;
import com.ctre.phoenix6.configs.Slot0Configs;
import com.ctre.phoenix6.controls.MotionMagicVoltage;
import com.ctre.phoenix6.controls.VoltageOut;
import com.ctre.phoenix6.hardware.TalonFX;
import com.ctre.phoenix6.signals.NeutralModeValue;

import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants.ClimberConstants;

public class Climber extends SubsystemBase {
    private final boolean climberEnabled = ClimberConstants.kClimberEnabled;
    private final TalonFX climberMotor;

    private final MotionMagicVoltage positionRequest = new MotionMagicVoltage(0);
    private final VoltageOut stopRequest = new VoltageOut(0);
    private double targetPositionRotations = ClimberConstants.kClimberDownRotations;

    public Climber() {
        if (climberEnabled) {
            climberMotor = new TalonFX(ClimberConstants.kClimberMotorID);
            configureMotor();
            seedPositionZero();
            setClimberPositionRotations(ClimberConstants.kClimberDownRotations);
        } else {
            climberMotor = null;
        }
    }

    public double getClimberPositionRotations() {
        if (!climberEnabled || climberMotor == null) {
            return ClimberConstants.kClimberDownRotations;
        }

        return climberMotor.getRotorPosition().getValueAsDouble();
    }

    public double getTargetPositionRotations() {
        return targetPositionRotations;
    }

    public boolean atTarget() {
        return Math.abs(getClimberPositionRotations() - targetPositionRotations)
            <= ClimberConstants.kClimberPositionToleranceRotations;
    }

    public void setClimberPositionRotations(double targetPositionRotations) {
        this.targetPositionRotations = targetPositionRotations;

        if (!climberEnabled || climberMotor == null) {
            return;
        }

        climberMotor.setControl(positionRequest.withPosition(targetPositionRotations));
    }

    public void stop() {
        if (!climberEnabled || climberMotor == null) {
            return;
        }

        climberMotor.setControl(stopRequest.withOutput(0));
    }

    public Command setClimberPositionCommand(double targetPositionRotations) {
        return runOnce(() -> setClimberPositionRotations(targetPositionRotations));
    }

    public Command raiseCommand() {
        return setClimberPositionCommand(ClimberConstants.kClimberUpRotations);
    }

    public Command lowerCommand() {
        return setClimberPositionCommand(ClimberConstants.kClimberDownRotations);
    }

    public Command stopCommand() {
        return runOnce(this::stop);
    }

    @Override
    public void periodic() {
        SmartDashboard.putBoolean("Climber Enabled", climberEnabled);
        SmartDashboard.putNumber("Climber Position Rotations", getClimberPositionRotations());
        SmartDashboard.putNumber("Climber Target Rotations", targetPositionRotations);
        SmartDashboard.putBoolean("Climber At Target", atTarget());
    }

    private void seedPositionZero() {
        climberMotor.setPosition(ClimberConstants.kClimberDownRotations);
    }

    private void configureMotor() {
        var motorOutputConfigs = new MotorOutputConfigs();
        motorOutputConfigs.NeutralMode = NeutralModeValue.Brake;

        var currentLimitConfigs = new CurrentLimitsConfigs();
        currentLimitConfigs.StatorCurrentLimit = ClimberConstants.kClimberCurrentLimit;
        currentLimitConfigs.StatorCurrentLimitEnable = true;

        var slot0Configs = new Slot0Configs();
        slot0Configs.kP = ClimberConstants.kClimberKP;
        slot0Configs.kI = ClimberConstants.kClimberKI;
        slot0Configs.kD = ClimberConstants.kClimberKD;
        slot0Configs.kG = ClimberConstants.kClimberKG;

        var motionMagicConfigs = new MotionMagicConfigs();
        motionMagicConfigs.MotionMagicCruiseVelocity =
            ClimberConstants.kClimberCruiseVelocityRotationsPerSecond;
        motionMagicConfigs.MotionMagicAcceleration =
            ClimberConstants.kClimberAccelerationRotationsPerSecondSquared;
        motionMagicConfigs.MotionMagicJerk =
            ClimberConstants.kClimberJerkRotationsPerSecondCubed;

        climberMotor.getConfigurator().apply(motorOutputConfigs);
        climberMotor.getConfigurator().apply(currentLimitConfigs);
        climberMotor.getConfigurator().apply(slot0Configs);
        climberMotor.getConfigurator().apply(motionMagicConfigs);
    }
}
