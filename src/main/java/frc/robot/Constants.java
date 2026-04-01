/*
 * Copyright (C) 2026 Team 3602 All rights reserved. This work is
 * licensed under the terms of the MIT license which can be found
 * in the root directory of this project.
 */

package frc.robot;

import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.util.Units;

public final class Constants {
    private Constants() {}

    public static final class ShooterConstants {
        private static final double kLegacyShooterCommandMaxVelocityRotationsPerSecond = 95.0;
        private static final double kLegacyShooterVelocityKP = 0.139;
        private static final double kLegacyShooterVelocityKI = 0.0;
        private static final double kLegacyShooterVelocityKD = 0.0;
        private static final double kLegacyShooterVelocityKS = 0.25;
        private static final double kLegacyShooterVelocityKV = 0.139;
        private static final double kLegacyShooterVelocityKA = 0.0;
        private static final double kLegacyShooterAccelerationRotationsPerSecondSquared = 600.0;
        private static final double kLegacyShooterJerkRotationsPerSecondCubed = 6000.0;
        private static final double kLegacyShooterFailsafeVelocityRotationsPerSecond = 37.5;
        private static final double kOverallShotVelocityScale = 1.20;
        private static final double kReferenceShotVelocityMagnitudeRotationsPerSecond =
            44.0 * kOverallShotVelocityScale;
        private static final double kReferenceFeedVelocityMagnitudeRotationsPerSecond =
            legacyMotorRpsToMechanismRps(kLegacyShooterCommandMaxVelocityRotationsPerSecond);
        private static final double kShooterGravityMetersPerSecondSquared = 9.80665;
        private static final double kShooterLaunchAngleRadians = Math.toRadians(66.0);
        private static final double kShooterLaunchCosine = Math.cos(kShooterLaunchAngleRadians);
        private static final double kShooterLaunchTangent = Math.tan(kShooterLaunchAngleRadians);

        //Motor ID
        public static final int kShooterMotor1ID = 5;
        public static final int kShooterMotor2ID = 6;

        // Legacy percent output and nominal closed-loop reference speed
        public static final double kShooterSpeed = .75;
        public static final double kShooterSensorToMechanismRatio = 22.0 / 18.0;
        public static final double kShooterTargetVelocityRotationsPerSecond =
            -kReferenceShotVelocityMagnitudeRotationsPerSecond;
        public static final double kShooterCommandMaxVelocityRotationsPerSecond = 95.0;

        public static final double kShooterCurrentLimit = 60.0;
        public static final double kShooterVelocityKP =
            legacyVelocityGainToMechanismUnits(kLegacyShooterVelocityKP);
        public static final double kShooterVelocityKI =
            legacyVelocityGainToMechanismUnits(kLegacyShooterVelocityKI);
        public static final double kShooterVelocityKD =
            legacyVelocityGainToMechanismUnits(kLegacyShooterVelocityKD);
        public static final double kShooterVelocityKS = kLegacyShooterVelocityKS;
        public static final double kShooterVelocityKV =
            legacyVelocityGainToMechanismUnits(kLegacyShooterVelocityKV);
        public static final double kShooterVelocityKA =
            legacyVelocityGainToMechanismUnits(kLegacyShooterVelocityKA);
        public static final double kShooterAccelerationRotationsPerSecondSquared =
            legacyMotorRpsToMechanismRps(kLegacyShooterAccelerationRotationsPerSecondSquared);
        public static final double kShooterJerkRotationsPerSecondCubed =
            legacyMotorRpsToMechanismRps(kLegacyShooterJerkRotationsPerSecondCubed);

        public static final double kShooterExitHeightMeters = Units.inchesToMeters(20.5);
        public static final double kHubCenterHeightMeters = Units.inchesToMeters(72.0);
        public static final double kShooterLaunchAngleDegrees = 66.0;
        public static final double kReferenceShotDistanceMeters = 2.98;
        public static final double kShooterMinimumLookaheadSeconds = 0.05;
        public static final double kReferenceShotLaunchVelocityMetersPerSecond =
            idealLaunchVelocityMetersPerSecondForDistanceMeters(kReferenceShotDistanceMeters);
        public static final double kShooterExitSpeedMetersPerSecondPerRotationPerSecond =
            kReferenceShotLaunchVelocityMetersPerSecond
                / kReferenceShotVelocityMagnitudeRotationsPerSecond;

        //Failsafe Speed
        public static final double kShooterFailsafeSpeed =
            legacyMotorRpsToMechanismRps(kLegacyShooterFailsafeVelocityRotationsPerSecond)
                * kOverallShotVelocityScale;
        public static final double kShooterReverseVelocityRotationsPerSecond =
            legacyMotorRpsToMechanismRps(kLegacyShooterFailsafeVelocityRotationsPerSecond);

        public static double velocityForDistanceMeters(double distanceMeters) {
            return -shooterVelocityMagnitudeForDistanceMeters(distanceMeters);
        }

        public static double ballTimeOfFlightSecondsForDistanceMeters(double distanceMeters) {
            double launchVelocityMetersPerSecond =
                launchVelocityMetersPerSecondForDistanceMeters(distanceMeters);
            double horizontalVelocityMetersPerSecond =
                launchVelocityMetersPerSecond * kShooterLaunchCosine;

            if (horizontalVelocityMetersPerSecond <= 1e-6) {
                return kShooterMinimumLookaheadSeconds;
            }

            return Math.max(
                kShooterMinimumLookaheadSeconds,
                distanceMeters / horizontalVelocityMetersPerSecond);
        }

        public static double launchVelocityMetersPerSecondForDistanceMeters(double distanceMeters) {
            return shooterVelocityMagnitudeForDistanceMeters(distanceMeters)
                * kShooterExitSpeedMetersPerSecondPerRotationPerSecond;
        }

        private static double legacyMotorRpsToMechanismRps(double legacyMotorRps) {
            return legacyMotorRps / kShooterSensorToMechanismRatio;
        }

        private static double legacyVelocityGainToMechanismUnits(double legacyGain) {
            return legacyGain * kShooterSensorToMechanismRatio;
        }

        private static double shooterVelocityMagnitudeForDistanceMeters(double distanceMeters) {
            double idealLaunchVelocityMetersPerSecond =
                idealLaunchVelocityMetersPerSecondForDistanceMeters(distanceMeters);

            if (!Double.isFinite(idealLaunchVelocityMetersPerSecond)
                    || idealLaunchVelocityMetersPerSecond <= 0.0) {
                return kReferenceShotVelocityMagnitudeRotationsPerSecond;
            }

            return idealLaunchVelocityMetersPerSecond
                / kShooterExitSpeedMetersPerSecondPerRotationPerSecond;
        }

        private static double idealLaunchVelocityMetersPerSecondForDistanceMeters(
                double distanceMeters) {
            double heightDeltaMeters = kHubCenterHeightMeters - kShooterExitHeightMeters;
            double denominator =
                2.0
                    * kShooterLaunchCosine
                    * kShooterLaunchCosine
                    * ((distanceMeters * kShooterLaunchTangent) - heightDeltaMeters);

            if (distanceMeters <= 0.0 || denominator <= 1e-6) {
                return Double.NaN;
            }

            return Math.sqrt(
                kShooterGravityMetersPerSecondSquared
                    * distanceMeters
                    * distanceMeters
                    / denominator);
        }
    }

    public static final class ClimberConstants {
        // Feature flag
        // Set this to true when the climber hardware is back on the robot and the
        // CAN ID below is correct. Leaving it false keeps the climber code in the
        // project without letting the robot talk to hardware that is not installed.
        public static final boolean kClimberEnabled = true;

        //Motor ID
        public static final int kClimberMotorID = 15;

        public static final double kClimberCurrentLimit = 40.0;
        public static final double kClimberKP = 8.0;
        public static final double kClimberKI = 0.0;
        public static final double kClimberKD = 0.0;
        public static final double kClimberKG = 0.0;
        public static final double kClimberCruiseVelocityRotationsPerSecond = 27.0;
        public static final double kClimberAccelerationRotationsPerSecondSquared = 54.0;
        public static final double kClimberJerkRotationsPerSecondCubed = 270.0;
        public static final double kClimberDownRotations = 0.0;
        public static final double kClimberUpRotations = 65;
        public static final double kClimberPositionToleranceRotations = 0.2;
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
        public static final double kPivotRaiseDegrees = 5.0;
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
        private static final double kTurretOutputStageRatio = 10.0;
        private static final double kLegacyTurretGearboxRatio = 9.0;
        private static final double kTurretGearboxRatio = 3.0;
        private static final double kLegacyTurretGearRatio =
            kLegacyTurretGearboxRatio * kTurretOutputStageRatio;
        private static final double kLegacyTurretKP = 18.0;
        private static final double kLegacyTurretCruiseVelocityRotationsPerSecond = 85.0;
        private static final double kLegacyTurretAccelerationRotationsPerSecondSquared = 335.0;
        private static final double kLegacyTurretJerkRotationsPerSecondCubed = 1670.0;

        //Motor ID
        public static final int kTurretMotorID = 9;
        public static final int kTurretEncoderID = 10;

        // The turret kept the same 10:1 output stage and swapped the gearbox from 9:1 to 3:1.
        public static final double kTurretGearRatio = kTurretGearboxRatio * kTurretOutputStageRatio;
        public static final double kTurretCurrentLimit = 50.0;
        public static final double kTurretStartAngleDegrees = -90.0;
        // Scale the position loop from the old 90:1 reduction so a degree of turret error still
        // produces about the same corrective effort after the gearbox swap.
        public static final double kTurretKP = kLegacyTurretKP
            * (kLegacyTurretGearRatio / kTurretGearRatio);
        public static final double kTurretKI = 0.0;
        public static final double kTurretKD = 0.0;
        public static final double kTurretKG = 0.0;
        // Keep the old motor-side Motion Magic limits so the 3:1 gearbox can translate them into
        // a much faster turret output speed than the previous 9:1 gearbox allowed.
        public static final double kTurretCruiseVelocityRotationsPerSecond =
            kLegacyTurretCruiseVelocityRotationsPerSecond;
        public static final double kTurretAccelerationRotationsPerSecondSquared =
            kLegacyTurretAccelerationRotationsPerSecondSquared;
        public static final double kTurretJerkRotationsPerSecondCubed =
            kLegacyTurretJerkRotationsPerSecondCubed;
        public static final double kTurretPositionToleranceDegrees = 2.0;
        public static final double kTurretShotToleranceDegrees = 5.0;
        public static final double kTurretProjectileSpeedMetersPerSecond = 12.0;
        public static final double kTurretMinimumLookaheadSeconds = 0.05;
        // Scales how far ahead we project the robot's future translation when solving the
        // shot angle. Larger values make the turret aim farther opposite the robot's
        // current drive direction so the note's inherited chassis velocity lands closer
        // to the tower while strafing.
        public static final double kTurretTranslationalLeadGain = 1.75;
        public static final double kTurretRotationalLeadGain = 1.0;
        // Scale the turret's Motion Magic profile up as chassis translation speed rises so
        // tracking stays snappy while driving, but remains tame when the robot is mostly still.
        public static final double kTurretDriveMotionVelocityScaleAtFullSpeed = 2.25;
        public static final double kTurretDriveMotionAccelerationScaleAtFullSpeed = 3.5;
        public static final double kTurretDriveMotionJerkScaleAtFullSpeed = 3.5;
        // Give the turret additional authority while the chassis is yawing so it can keep up
        // with a continuously moving field-relative target instead of catching up after rotation
        // stops. This matches the driver's configured 0.75 rotations/sec max turn rate.
        public static final double kTurretMaxChassisAngularRateRadiansPerSecond =
            0.75 * 2.0 * Math.PI;
        public static final double kTurretRotateMotionVelocityScaleAtFullYawRate = 2.0;
        public static final double kTurretRotateMotionAccelerationScaleAtFullYawRate = 3.0;
        public static final double kTurretRotateMotionJerkScaleAtFullYawRate = 3.0;
        public static final double kCenterFieldXMinMeters = 6.0;
        public static final double kCenterFieldXMaxMeters = 10.5;
    }

    public static final class SpindexerConstants {
        //Motor ID
        public static final int kSpindexerMotorID = 12;
        public static final int kReceiveMotorID = 11;

        // Feed path ratios off shooter target speed.
        public static final double kSpindexerSpeedRatioToShooter = 0.54;
        public static final double kReceiverSpeedRatioToShooter = 0.432;
        public static final double kSpindexerVelocityRotationsPerSecond =
            -ShooterConstants.kReferenceFeedVelocityMagnitudeRotationsPerSecond
                * kSpindexerSpeedRatioToShooter;
        public static final double kReceiverVelocityRotationsPerSecond =
            -ShooterConstants.kReferenceFeedVelocityMagnitudeRotationsPerSecond
                * kReceiverSpeedRatioToShooter;

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
