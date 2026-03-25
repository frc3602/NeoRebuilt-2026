package frc.robot.subsystems;

import com.ctre.phoenix6.configs.CurrentLimitsConfigs;
import com.ctre.phoenix6.configs.MotorOutputConfigs;
import com.ctre.phoenix6.controls.VoltageOut;
import com.ctre.phoenix6.hardware.TalonFX;
import com.ctre.phoenix6.signals.NeutralModeValue;

import edu.wpi.first.math.controller.ArmFeedforward;
import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants.ClimberConstants;

public class Climber extends SubsystemBase {
    private final boolean climberEnabled = ClimberConstants.kClimberEnabled;
    private final TalonFX climberMotor;

    private final PIDController positionController = new PIDController(
        ClimberConstants.kClimberPositionKP,
        ClimberConstants.kClimberPositionKI,
        ClimberConstants.kClimberPositionKD);
    private final ArmFeedforward feedforward = new ArmFeedforward(
        ClimberConstants.kClimberFeedforwardKS,
        ClimberConstants.kClimberFeedforwardKG,
        ClimberConstants.kClimberFeedforwardKV,
        ClimberConstants.kClimberFeedforwardKA);
    private final VoltageOut voltageRequest = new VoltageOut(0);

    private double targetHeightInches = ClimberConstants.kClimberLowerHeightInches;
    private double pidEffortVolts = 0.0;
    private double feedforwardEffortVolts = 0.0;
    private double appliedVoltageVolts = 0.0;

    public Climber() {
        if (climberEnabled) {
            climberMotor = new TalonFX(ClimberConstants.kClimberMotorID);
            configureMotor();
            stop();
        } else {
            climberMotor = null;
        }

        positionController.setTolerance(ClimberConstants.kClimberHeightToleranceInches);
    }

    public double getHeightInches() {
        if (!climberEnabled || climberMotor == null) {
            return 0.0;
        }

        return rotorRotationsToHeightInches(climberMotor.getRotorPosition().getValueAsDouble());
    }

    public double getTargetHeightInches() {
        return targetHeightInches;
    }

    public double getPidEffortVolts() {
        return pidEffortVolts;
    }

    public double getFeedforwardEffortVolts() {
        return feedforwardEffortVolts;
    }

    public double getAppliedVoltageVolts() {
        return appliedVoltageVolts;
    }

    public boolean isAtTargetHeight() {
        if (!climberEnabled) {
            return true;
        }

        return positionController.atSetpoint();
    }

    public void setHeightInches(double targetHeightInches) {
        this.targetHeightInches = targetHeightInches;
    }

    public void stop() {
        pidEffortVolts = 0.0;
        feedforwardEffortVolts = 0.0;
        appliedVoltageVolts = 0.0;

        if (!climberEnabled || climberMotor == null) {
            return;
        }

        climberMotor.setControl(voltageRequest.withOutput(0));
    }

    public Command setHeightCommand(double targetHeightInches) {
        return runOnce(() -> setHeightInches(targetHeightInches));
    }

    public Command raiseCommand() {
        return setHeightCommand(ClimberConstants.kClimberRaiseHeightInches);
    }

    public Command lowerCommand() {
        return setHeightCommand(ClimberConstants.kClimberLowerHeightInches);
    }

    public Command stopCommand() {
        return runOnce(this::stop);
    }

    @Override
    public void periodic() {
        SmartDashboard.putBoolean("Climber Enabled", climberEnabled);
        SmartDashboard.putNumber("Climber Target Height Inches", targetHeightInches);
        SmartDashboard.putNumber("Climber Height Inches", getHeightInches());
        SmartDashboard.putBoolean("Climber At Target", isAtTargetHeight());

        if (!climberEnabled || climberMotor == null) {
            return;
        }

        pidEffortVolts = positionController.calculate(getHeightInches(), targetHeightInches);
        feedforwardEffortVolts = feedforward.calculate(0.0, 0.0);
        appliedVoltageVolts = pidEffortVolts + feedforwardEffortVolts;

        climberMotor.setControl(voltageRequest.withOutput(appliedVoltageVolts));
    }

    private void configureMotor() {
        var motorOutputConfigs = new MotorOutputConfigs();
        motorOutputConfigs.NeutralMode = NeutralModeValue.Brake;

        var currentLimitConfigs = new CurrentLimitsConfigs();
        currentLimitConfigs.StatorCurrentLimit = ClimberConstants.kClimberCurrentLimit;
        currentLimitConfigs.StatorCurrentLimitEnable = true;

        climberMotor.getConfigurator().apply(motorOutputConfigs);
        climberMotor.getConfigurator().apply(currentLimitConfigs);
    }

    private double rotorRotationsToHeightInches(double rotorRotations) {
        return (rotorRotations / ClimberConstants.kClimberGearRatio)
            * (Math.PI * ClimberConstants.kClimberSprocketDiameterInches);
    }
}
