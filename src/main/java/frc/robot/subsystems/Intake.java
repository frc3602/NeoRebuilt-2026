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
import frc.robot.Constants.IntakeConstants;

public class Intake extends SubsystemBase {
    private final TalonFX intakeMotor = new TalonFX(IntakeConstants.kIntakeMotorID);

    private final MotionMagicVelocityVoltage velocityRequest = new MotionMagicVelocityVoltage(0);
    private final VoltageOut stopRequest = new VoltageOut(0);
    private double targetVelocityRotationsPerSecond = 0.0;

    public Intake() {
        configureMotor();
        stop();
    }

    public void intakeForward() {
        targetVelocityRotationsPerSecond = IntakeConstants.kIntakeForwardVelocityRotationsPerSecond;
        intakeMotor.setControl(
            velocityRequest.withVelocity(targetVelocityRotationsPerSecond));
    }

    public void intakeReverse() {
        targetVelocityRotationsPerSecond = IntakeConstants.kIntakeReverseVelocityRotationsPerSecond;
        intakeMotor.setControl(
            velocityRequest.withVelocity(targetVelocityRotationsPerSecond));
    }

    public void stop() {
        targetVelocityRotationsPerSecond = 0.0;
        intakeMotor.setControl(stopRequest.withOutput(0));
    }

    public double getTargetVelocityRotationsPerSecond() {
        return targetVelocityRotationsPerSecond;
    }

    public double getMeasuredVelocityRotationsPerSecond() {
        return intakeMotor.getVelocity().getValueAsDouble();
    }

    public Command intakeForwardCommand() {
        return runOnce(this::intakeForward);
    }

    public Command intakeReverseCommand() {
        return runOnce(this::intakeReverse);
    }

    public Command stopCommand() {
        return runOnce(this::stop);
    }

    private void configureMotor() {
        var motorOutputConfigs = new MotorOutputConfigs();
        motorOutputConfigs.NeutralMode = NeutralModeValue.Brake;

        var currentLimitConfigs = new CurrentLimitsConfigs();
        currentLimitConfigs.StatorCurrentLimit = IntakeConstants.kIntakeCurrentLimit;
        currentLimitConfigs.StatorCurrentLimitEnable = true;

        var slot0Configs = new Slot0Configs();
        slot0Configs.kP = IntakeConstants.kIntakeVelocityKP;
        slot0Configs.kI = IntakeConstants.kIntakeVelocityKI;
        slot0Configs.kD = IntakeConstants.kIntakeVelocityKD;
        slot0Configs.kS = IntakeConstants.kIntakeVelocityKS;
        slot0Configs.kV = IntakeConstants.kIntakeVelocityKV;
        slot0Configs.kA = IntakeConstants.kIntakeVelocityKA;

        var motionMagicConfigs = new MotionMagicConfigs();
        motionMagicConfigs.MotionMagicAcceleration =
            IntakeConstants.kIntakeAccelerationRotationsPerSecondSquared;
        motionMagicConfigs.MotionMagicJerk =
            IntakeConstants.kIntakeJerkRotationsPerSecondCubed;

        intakeMotor.getConfigurator().apply(motorOutputConfigs);
        intakeMotor.getConfigurator().apply(currentLimitConfigs);
        intakeMotor.getConfigurator().apply(slot0Configs);
        intakeMotor.getConfigurator().apply(motionMagicConfigs);
    }
}
