package frc.robot.subsystems;

import java.util.function.Supplier;

import com.ctre.phoenix6.Utils;
import com.ctre.phoenix6.swerve.SwerveDrivetrainConstants;
import com.ctre.phoenix6.swerve.SwerveModuleConstants;
import com.ctre.phoenix6.swerve.SwerveRequest;

import com.pathplanner.lib.auto.AutoBuilder;
import com.pathplanner.lib.config.PIDConstants;
import com.pathplanner.lib.config.RobotConfig;
import com.pathplanner.lib.controllers.PPHolonomicDriveController;

import edu.wpi.first.math.VecBuilder;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.DriverStation.Alliance;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Subsystem;

import frc.robot.generated.TunerConstants;
import frc.robot.generated.TunerConstants.TunerSwerveDrivetrain;
import frc.robot.subsystems.Limelight_Pose.AcceptedVisionMeasurement;

public class Drivetrain extends TunerSwerveDrivetrain implements Subsystem {

    private final SwerveRequest.ApplyRobotSpeeds autoRobotDrive = new SwerveRequest.ApplyRobotSpeeds();
    private final Limelight_Pose limelightPose = Limelight_Pose.getInstance();
    private long visionPoseUpdateCount = 0;

    public Drivetrain(
            SwerveDrivetrainConstants drivetrainConstants,
            SwerveModuleConstants<?, ?, ?>... modules) {
        super(drivetrainConstants, modules);

        if (Utils.isSimulation()) {
            startSimThread();
        }
    }

    public Drivetrain(
            SwerveDrivetrainConstants drivetrainConstants,
            double odometryUpdateFrequency,
            SwerveModuleConstants<?, ?, ?>... modules) {
        super(drivetrainConstants, odometryUpdateFrequency, modules);

        if (Utils.isSimulation()) {
            startSimThread();
        }
    }

    public Command applyRequest(Supplier<SwerveRequest> request) {
        return run(() -> this.setControl(request.get()));
    }

    public Pose2d getEstimatedPose() {
        return getState().Pose;
    }

    public long getVisionPoseUpdateCount() {
        return visionPoseUpdateCount;
    }

    public void applyDriverRequestedVisionSnap() {
        Pose2d visionSnapPose = limelightPose.getDriverRequestedVisionSnapPose();
        if (visionSnapPose == null) {
            DriverStation.reportWarning(
                "Driver requested Limelight pose snap, but no accepted vision pose was available.", false);
            return;
        }

        resetPose(visionSnapPose);
        limelightPose.markDrivePoseVisionResetApplied();
        visionPoseUpdateCount++;
    }

    public Command applyDriverRequestedVisionSnapCommand() {
        return runOnce(this::applyDriverRequestedVisionSnap);
    }

    public void configPathplanner() {
        RobotConfig robotConfig;

        try {
            robotConfig = RobotConfig.fromGUISettings();
        } catch (Exception ex) {
            DriverStation.reportWarning(
                "PathPlanner GUI settings missing; using drivetrain fallback config.", false);
            robotConfig = TunerConstants.createPathPlannerConfig();
        }

        AutoBuilder.configure(
            this::getEstimatedPose,
            this::resetPose,
            () -> getState().Speeds,
            (speeds, feedforwards) -> setControl(
                autoRobotDrive.withSpeeds(speeds)
                    .withWheelForceFeedforwardsX(feedforwards.robotRelativeForcesXNewtons())
                    .withWheelForceFeedforwardsY(feedforwards.robotRelativeForcesYNewtons())
            ),
            new PPHolonomicDriveController(
                new PIDConstants(10),
                new PIDConstants(7)
            ),
            robotConfig,
            () -> DriverStation.getAlliance().orElse(Alliance.Blue) == Alliance.Red,
            this
        );
    }

    @Override
    public void periodic() {
        var state = getState();
        double linearSpeedMetersPerSecond = Math.hypot(
            state.Speeds.vxMetersPerSecond,
            state.Speeds.vyMetersPerSecond
        );
        double yawRateDegreesPerSecond = Math.toDegrees(state.Speeds.omegaRadiansPerSecond);

        limelightPose.CollectDriveState(
            getEstimatedPose(),
            yawRateDegreesPerSecond,
            linearSpeedMetersPerSecond
        );

        var acceptedMeasurements = limelightPose.getAcceptedMeasurements();

        for (AcceptedVisionMeasurement measurement : acceptedMeasurements) {
            addVisionMeasurement(
                measurement.poseEstimate.pose,
                measurement.poseEstimate.timestampSeconds,
                VecBuilder.fill(
                    measurement.xyStdDev,
                    measurement.xyStdDev,
                    measurement.thetaStdDev
                )
            );
            visionPoseUpdateCount++;
        }

        Pose2d visionResetCandidate = limelightPose.getDrivePoseVisionResetCandidate();
        if (visionResetCandidate != null) {
            resetPose(visionResetCandidate);
            limelightPose.markDrivePoseVisionResetApplied();
            visionPoseUpdateCount++;
        }

        if (!acceptedMeasurements.isEmpty()) {
            limelightPose.UpdateVisionCorrectionAdded();
        }
    }

    /* ---------------- Simulation ---------------- */

    private static final double kSimLoopPeriod = 0.004;
    private edu.wpi.first.wpilibj.Notifier m_simNotifier;
    private double m_lastSimTime;

    private void startSimThread() {
        m_lastSimTime = Utils.getCurrentTimeSeconds();

        m_simNotifier = new edu.wpi.first.wpilibj.Notifier(() -> {
            double currentTime = Utils.getCurrentTimeSeconds();
            double deltaTime = currentTime - m_lastSimTime;
            m_lastSimTime = currentTime;

            updateSimState(deltaTime, edu.wpi.first.wpilibj.RobotController.getBatteryVoltage());
        });

        m_simNotifier.startPeriodic(kSimLoopPeriod);
    }
}
