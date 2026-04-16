package frc.robot.subsystems;

import com.ctre.phoenix6.configs.CurrentLimitsConfigs;
import com.ctre.phoenix6.configs.MotionMagicConfigs;
import com.ctre.phoenix6.configs.MotorOutputConfigs;
import com.ctre.phoenix6.configs.Slot0Configs;
import com.ctre.phoenix6.controls.Follower;
import com.ctre.phoenix6.controls.MotionMagicVoltage;
import com.ctre.phoenix6.controls.StaticBrake;
import com.ctre.phoenix6.controls.VoltageOut;
import com.ctre.phoenix6.hardware.TalonFX;
import com.ctre.phoenix6.signals.GravityTypeValue;
import com.ctre.phoenix6.signals.MotorAlignmentValue;
import com.ctre.phoenix6.signals.NeutralModeValue;

import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants.IntakeConstants;

public class Pivot extends SubsystemBase {
    private final TalonFX intakePivot = new TalonFX(IntakeConstants.kPivotLeaderMotorID);
    private final TalonFX intakePivotFollow = new TalonFX(IntakeConstants.kPivotFollowerMotorID);

    private final MotionMagicVoltage motionMagicRequest = new MotionMagicVoltage(0);
    private final VoltageOut manualVoltageRequest = new VoltageOut(0);
    private final Follower followerRequest =
        new Follower(intakePivot.getDeviceID(), MotorAlignmentValue.Opposed);

    private double pivotSetpointDegrees = 0.0;

    public Pivot() {
        intakePivot.setPosition(0);
        intakePivotFollow.setPosition(0);

        var motorConfig = new MotorOutputConfigs();
        motorConfig.NeutralMode = NeutralModeValue.Brake;

        var limitConfig = new CurrentLimitsConfigs();
        limitConfig.StatorCurrentLimit = IntakeConstants.kPivotCurrentLimit;
        limitConfig.StatorCurrentLimitEnable = true;

        var slot0Config = new Slot0Configs();
        slot0Config.kP = IntakeConstants.kPivotKP;
        slot0Config.kI = IntakeConstants.kPivotKI;
        slot0Config.kD = IntakeConstants.kPivotKD;
        slot0Config.kG = IntakeConstants.kPivotKG;
        slot0Config.GravityType = GravityTypeValue.Arm_Cosine;

        var motionMagicConfig = new MotionMagicConfigs();
        motionMagicConfig.MotionMagicCruiseVelocity =
            IntakeConstants.kPivotCruiseVelocityRotationsPerSecond;
        motionMagicConfig.MotionMagicAcceleration =
            IntakeConstants.kPivotAccelerationRotationsPerSecondSquared;
        motionMagicConfig.MotionMagicJerk = IntakeConstants.kPivotJerkRotationsPerSecondCubed;

        

        intakePivot.getConfigurator().apply(motorConfig);
        intakePivot.getConfigurator().apply(limitConfig);
        intakePivot.getConfigurator().apply(slot0Config);
        intakePivot.getConfigurator().apply(motionMagicConfig);

        intakePivotFollow.getConfigurator().apply(motorConfig);
        intakePivotFollow.getConfigurator().apply(limitConfig);
        intakePivotFollow.setControl(followerRequest);

        setPivotPositionDegrees(0.0);
    }

    public double getRightPosition() {
        return rotorRotationsToDegrees(intakePivot.getRotorPosition().getValueAsDouble());
    }

    public double getLeftPosition() {
        return -rotorRotationsToDegrees(intakePivotFollow.getRotorPosition().getValueAsDouble());
    }

    public double getPivotSetpointDegrees() {
        return pivotSetpointDegrees;
    }

    public void setPivotPositionDegrees(double targetDegrees) {
        pivotSetpointDegrees = targetDegrees;
        intakePivot.setControl(motionMagicRequest.withPosition(degreesToRotorRotations(targetDegrees)));
    }

    public Command setPivotPositionCommand(double targetDegrees) {
        return runOnce(() -> setPivotPositionDegrees(targetDegrees));
    }

    public Command smartRaisePivot() {
        return setPivotPositionCommand(pivotSetpointDegrees - IntakeConstants.kPivotSmartStepDegrees);
    }

    public Command smartDropPivot() {
        return setPivotPositionCommand(pivotSetpointDegrees + IntakeConstants.kPivotSmartStepDegrees);
    }

    public Command dumbDropIntake() {
        return setPivotPositionCommand(IntakeConstants.kPivotDropDegrees);
    }

    public Command dumbRaiseIntake() {
        return setPivotPositionCommand(IntakeConstants.kPivotRaiseDegrees);
    }

    public Command dumbPartialRaiseIntake() {
        return setPivotPositionCommand(IntakeConstants.kPartialPivotRaiseDegrees);
    }

    public boolean isRightDown() {
        return getRightPosition() > 90.0;
    }

    public boolean isRightUP() {
        return getRightPosition() < 0.0;
    }

    public Command runRightPivot(double volts) {
        return startEnd(
            () -> intakePivot.setControl(manualVoltageRequest.withOutput(volts)),
            () -> setPivotPositionDegrees(getRightPosition())
        );
    }

    public Command runLeftPivot(double volts) {
        return runRightPivot(volts);
    }

    private double degreesToRotorRotations(double degrees) {
        return (degrees / 360.0) * IntakeConstants.kPivotGearRatio;
    }

    private double rotorRotationsToDegrees(double rotorRotations) {
        return (rotorRotations / IntakeConstants.kPivotGearRatio) * 360.0;
    }
}
