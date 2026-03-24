/*
 * Copyright (C) 2026 Team 3602 All rights reserved. This work is
 * licensed under the terms of the MIT license which can be found
 * in the root directory of this project.
 */

package frc.robot;

import edu.wpi.first.math.MathUtil;
import edu.wpi.first.math.geometry.Translation2d;

public final class Constants {
    private Constants() {}

    public static final class ShooterConstants {
        //Motor ID
        public static final int kShooterMotor1ID = 5;
        public static final int kShooterMotor2ID = 6;

        // Legacy percent output and nominal closed-loop reference speed
        public static final double kShooterSpeed = .75;
        public static final double kShooterTargetVelocityRotationsPerSecond = -100.0;

        public static final double kShooterCurrentLimit = 60.0;
        public static final double kShooterVelocityKP = 0.11;
        public static final double kShooterVelocityKI = 0.0;
        public static final double kShooterVelocityKD = 0.0;
        public static final double kShooterVelocityKS = 0.25;
        public static final double kShooterVelocityKV = 0.19;
        public static final double kShooterVelocityKA = 0.0;
        public static final double kShooterAccelerationRotationsPerSecondSquared = 600.0;
        public static final double kShooterJerkRotationsPerSecondCubed = 6000.0;

        // Distance-to-speed placeholder model for later calibration.
        public static final double kReferenceShotDistanceMeters = 3.0;
        public static final double kShooterMinVelocityRotationsPerSecond = -130.0;
        public static final double kShooterMaxVelocityRotationsPerSecond = -70.0;
        public static final double kShooterVelocitySlopeRotationsPerSecondPerMeter = -9.0;

        //Failsafe Speed
        public static final double kShooterFailsafeSpeed = 37.5;
        public static final double kShooterReverseVelocityRotationsPerSecond = 37.5;

        public static double velocityForDistanceMeters(double distanceMeters) {
            double targetVelocity = kShooterTargetVelocityRotationsPerSecond
                + (distanceMeters - kReferenceShotDistanceMeters)
                    * kShooterVelocitySlopeRotationsPerSecondPerMeter;
            return MathUtil.clamp(
                targetVelocity,
                kShooterMinVelocityRotationsPerSecond,
                kShooterMaxVelocityRotationsPerSecond);
        }
    }

    public static final class ClimberConstants {
        // Feature flag
        // Set this to true when the climber hardware is back on the robot and the
        // CAN ID below is correct. Leaving it false keeps the climber code in the
        // project without letting the robot talk to hardware that is not installed.
        public static final boolean kClimberEnabled = false;

        //Motor ID
        public static final int kClimberMotorID = 15;
    }

    public static final class OIConstants {
        public static final int kDriverControllerPort = 0;
        public static final int kOperatorControllerPort = 1;
    }

    public static final class IntakeConstants {
        //Motor ID
        public static final int kIntakeMotorID = 8;
        public static final int kPivotLeaderMotorID = 13;
        public static final int kPivotFollowerMotorID = 14;

        // Intake roller Motion Magic velocity tuning
        public static final double kIntakeCurrentLimit = 40.0;
        public static final double kIntakeForwardVelocityRotationsPerSecond = -70.0;
        public static final double kIntakeReverseVelocityRotationsPerSecond = 70.0;
        public static final double kIntakeVelocityKP = 0.12;
        public static final double kIntakeVelocityKI = 0.0;
        public static final double kIntakeVelocityKD = 0.0;
        public static final double kIntakeVelocityKS = 0.0;
        public static final double kIntakeVelocityKV = 0.0;
        public static final double kIntakeVelocityKA = 0.0;
        public static final double kIntakeAccelerationRotationsPerSecondSquared = 120.0;
        public static final double kIntakeJerkRotationsPerSecondCubed = 0.0;

        // Pivot limits and Motion Magic tuning
        public static final double kPivotCurrentLimit = 10;
        public static final double kPivotGearRatio = 125.0;
        public static final double kPivotSmartStepDegrees = 90.0;
        public static final double kPivotRaiseDegrees = 20.0;
        public static final double kPivotDropDegrees = 104.0;
        public static final double kPivotKP = 14.0;
        public static final double kPivotKI = 0.0;
        public static final double kPivotKD = 0.0;
        public static final double kPivotKG = 0.0;
        public static final double kPivotCruiseVelocityRotationsPerSecond = 35.0;
        public static final double kPivotAccelerationRotationsPerSecondSquared = 80.0;
        public static final double kPivotJerkRotationsPerSecondCubed = 400.0;

    }

    public static final class TurretConstants {
        //Motor ID
        public static final int kTurretMotorID = 9;
        public static final int kTurretEncoderID = 10;

        public static final double kTurretGearRatio = 90.0;
        public static final double kTurretCurrentLimit = 50.0;
        public static final double kTurretStartAngleDegrees = -90.0;
        public static final double kTurretKP = 18.0;
        public static final double kTurretKI = 0.0;
        public static final double kTurretKD = 0.0;
        public static final double kTurretKG = 0.0;
        public static final double kTurretCruiseVelocityRotationsPerSecond = 85.0;
        public static final double kTurretAccelerationRotationsPerSecondSquared = 335.0;
        public static final double kTurretJerkRotationsPerSecondCubed = 1670.0;
        public static final double kTurretPositionToleranceDegrees = 2.0;
        public static final double kTurretProjectileSpeedMetersPerSecond = 12.0;
        public static final double kTurretMinimumLookaheadSeconds = 0.05;
        public static final double kTurretTranslationalLeadGain = 1.0;
        public static final double kTurretRotationalLeadGain = 1.0;
        public static final double kCenterFieldXMinMeters = 6.0;
        public static final double kCenterFieldXMaxMeters = 10.5;
    }

    public static final class SpindexerConstants {
        //Motor ID
        public static final int kSpindexerMotorID = 12;
        public static final int kReceiveMotorID = 11;

        // Feed path ratios off shooter target speed:
        // spindexer is slowest, receiver is faster, shooter is fastest.
        public static final double kSpindexerSpeedRatioToShooter = 0.45;
        public static final double kReceiverSpeedRatioToShooter = 0.60;
        public static final double kSpindexerVelocityRotationsPerSecond =
            ShooterConstants.kShooterTargetVelocityRotationsPerSecond * kSpindexerSpeedRatioToShooter;
        public static final double kReceiverVelocityRotationsPerSecond =
            ShooterConstants.kShooterTargetVelocityRotationsPerSecond * kReceiverSpeedRatioToShooter;

        public static final double kSpindexerCurrentLimit = 40.0;
        public static final double kReceiverCurrentLimit = 40.0;
        public static final double kFeedVelocityKP = 0.11;
        public static final double kFeedVelocityKI = 0.0;
        public static final double kFeedVelocityKD = 0.0;
        public static final double kFeedVelocityKS = 0.25;
        public static final double kFeedVelocityKV = 0.12;
        public static final double kFeedVelocityKA = 0.10;
        public static final double kFeedAccelerationRotationsPerSecondSquared = 400.0;
        public static final double kFeedJerkRotationsPerSecondCubed = 6000.0;
    }

    public static final class FieldConstants {
        // Official REBUILT field coordinate system:
        // - (0, 0) is the blue-right corner when viewed from above
        // - +X points toward the red alliance wall
        // - +Y points left across the field
        //
        // These tower positions are the team's measured field targets in meters.
        // Keeping the measured numbers here lets turret aiming, distance
        // calculations, and autonomous tower shots all use the same real-world
        // reference points.
        public static final double kFieldLengthMeters = 16.55;
        public static final double kFieldWidthMeters = 8.05;
        public static final Translation2d kBlueTowerPosition = new Translation2d(4.634, 4.029);
        public static final Translation2d kRedTowerPosition = new Translation2d(11.919, 4.029);
        public static final Translation2d kBlueHubPosition = kBlueTowerPosition;
        public static final Translation2d kRedHubPosition = kRedTowerPosition;
        public static final Translation2d kBluePassCornerPosition = new Translation2d(0.8, 7.25);
        public static final Translation2d kRedPassCornerPosition =
            new Translation2d(kFieldLengthMeters - 0.8, kFieldWidthMeters - 7.25);
    }
}
