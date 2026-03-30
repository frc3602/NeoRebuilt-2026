// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import java.util.ArrayList;
import java.util.List;
import frc.robot.LimelightHelpers;
import frc.robot.LimelightHelpers.PoseEstimate;
import frc.robot.LimelightHelpers.RawFiducial;

public class Limelight_Pose extends SubsystemBase {
  /**
   * This small helper object stores how one camera frame was evaluated.
   *
   * We keep the data together so students can see the full decision in one place:
   * what the camera saw, whether we accepted it, which pose mode we chose, and how
   * much we trust that pose update.
   */
  private static class CameraMeasurementDecision {
    public String cameraName = "Unknown";
    public boolean processedFreshFrame = false;
    public boolean acceptedMeasurement = false;
    public boolean usingMegaTag1 = false;
    public boolean usingMegaTag2 = false;
    public PoseEstimate selectedEstimate = null;
    public double xyStdDev = 0.0;
    public double thetaStdDev = LARGE_ROTATION_STD_DEV;
    public int tagCount = 0;
    public double avgTagArea = 0.0;
    public double avgTagDist = 0.0;
    public double tagSpan = 0.0;
    public double maxAmbiguity = 0.0;
    public double qualityScore = Double.NEGATIVE_INFINITY;
    public double timestampSeconds = 0.0;
    public double measurementAgeSeconds = 0.0;
    public double translationErrorMeters = 0.0;
    public double headingErrorDegrees = 0.0;
    public String statusMessage = "No measurement processed yet";
  }

  /**
   * This small data object describes one accepted vision measurement that is ready
   * to be fused into the drivetrain estimator.
   *
   * We keep the estimator inputs bundled together so the drivetrain can apply both
   * accepted camera updates without re-computing trust values or re-reading
   * Limelight data.
   */
  public static class AcceptedVisionMeasurement {
    public final String cameraName;
    public final PoseEstimate poseEstimate;
    public final double xyStdDev;
    public final double thetaStdDev;

    public AcceptedVisionMeasurement(String cameraName, PoseEstimate poseEstimate,
        double xyStdDev, double thetaStdDev) {
      this.cameraName = cameraName;
      this.poseEstimate = poseEstimate;
      this.xyStdDev = xyStdDev;
      this.thetaStdDev = thetaStdDev;
    }
  }

  private static final String CAMERA_RIGHT = "limelight-right";
  private static final String CAMERA_LEFT = "limelight-left";

  // Smaller standard deviation means "trust this measurement more" in the pose
  // estimator. This branch intentionally uses fixed trust values so the vision
  // pipeline is easier to reason about and tune.
  private static final double LARGE_ROTATION_STD_DEV = 999999999.0;
  private static final double FIXED_XY_STD_DEV = 0.90;
  private static final double FIXED_THETA_STD_DEV = LARGE_ROTATION_STD_DEV;

  // This branch uses MegaTag2 only. We keep a few simple safety gates so each
  // camera can contribute translation help without the old MT1-vs-MT2 arbitration.
  private static final double MIN_TAG_AREA = 0.10;
  private static final double MAX_TAG_DISTANCE_METERS = 6.5;
  private static final double MAX_AMBIGUITY = 0.50;
  private static final double MAX_LATENCY_MILLISECONDS = 250.0;
  private static final double MAX_MEASUREMENT_AGE_SECONDS = 0.35;
  private static final double MAX_TRANSLATION_JUMP_METERS = 5.5;
  private static final double MIN_SINGLE_TAG_AREA = 0.10;
  private static final double MAX_SINGLE_TAG_DISTANCE_METERS = 4.5;
  private static final double MAX_SINGLE_TAG_AMBIGUITY = 0.40;

  /** Creates a new Limelight pose subsystem. */

  // Making a static instance of the subsystem lets other classes share the latest
  // accepted vision pose without building duplicate Limelight readers.
  private static Limelight_Pose _instance;

  /**
   * Returns the shared Limelight pose subsystem instance.
   *
   * We use a single instance so all code paths read the same camera decision state.
   */
  public static Limelight_Pose getInstance() {
    if (_instance == null) {
      _instance = new Limelight_Pose();
    }
    return _instance;
  }

  public boolean poseUpdatesFromCameraActive = true;

  public PoseEstimate poseCamEstimate;
  public double poseUpdateXYTrustFactor = 0;
  public double poseUpdateRotTrustFactor = LARGE_ROTATION_STD_DEV;
  public boolean poseUpdateAvailable = false;

  private boolean poseUpdateAvailableCam1 = false;
  private boolean poseUpdateAvailableCam2 = false;
  private boolean usingCam1MT1 = false;
  private boolean usingCam2MT1 = false;
  private boolean usingCam1MT2 = false;
  private boolean usingCam2MT2 = false;

  private double timestampCam1Previous = 0.0;
  private double timestampCam2Previous = 0.0;

  private Pose2d currentDrivePose = Pose2d.kZero;
  private boolean driveStateAvailable = false;
  public double currentDriveTheta;
  public double currentDriveYawRate;
  public double currentDriveLinearSpeedMetersPerSecond;
  private String selectedCameraName = "None";

  private CameraMeasurementDecision cam1Decision = new CameraMeasurementDecision();
  private CameraMeasurementDecision cam2Decision = new CameraMeasurementDecision();

  /** Creates the subsystem object that manages Limelight pose updates. */
  public Limelight_Pose() {
  }

  /**
   * Enables pose updates from the Limelights.
   *
   * This is useful when we want the robot to resume vision corrections after they
   * have been temporarily disabled for testing or debugging.
   */
  public void SetPoseCameraActive() {
    poseUpdatesFromCameraActive = true;
  }

  /**
   * Disables pose updates from the Limelights.
   *
   * When disabled, the drivetrain will rely only on onboard odometry and gyro
   * measurements until vision is turned back on.
   */
  public void ClearPoseCameraActive() {
    poseUpdatesFromCameraActive = false;
  }

  /**
   * Evaluates the right Limelight and stores whether it produced a fresh, useful
   * pose update this loop.
   *
   * The simplified branch treats the right camera as one independent MegaTag2 pose
   * source. If the frame is fresh and passes a few basic checks, it can be fused.
   */
  public void SetPoseEstimateInfoCam1() {
    cam1Decision = evaluateCameraMeasurement(CAMERA_RIGHT, timestampCam1Previous);
    if (cam1Decision.processedFreshFrame) {
      timestampCam1Previous = cam1Decision.timestampSeconds;
    }

    poseUpdateAvailableCam1 = cam1Decision.acceptedMeasurement;
    usingCam1MT1 = cam1Decision.usingMegaTag1;
    usingCam1MT2 = cam1Decision.usingMegaTag2;
  }

  /**
   * Evaluates the left Limelight and stores whether it produced a fresh, useful
   * pose update this loop.
   *
   * The method mirrors the right-camera logic so both cameras are judged with the
   * same simple rules and can be fused independently.
   */
  public void SetPoseEstimateInfoCam2() {
    cam2Decision = evaluateCameraMeasurement(CAMERA_LEFT, timestampCam2Previous);
    if (cam2Decision.processedFreshFrame) {
      timestampCam2Previous = cam2Decision.timestampSeconds;
    }

    poseUpdateAvailableCam2 = cam2Decision.acceptedMeasurement;
    usingCam2MT1 = cam2Decision.usingMegaTag1;
    usingCam2MT2 = cam2Decision.usingMegaTag2;
  }

  /**
   * Chooses the best accepted camera measurement to publish as the shared
   * "selected" vision pose.
   *
   * The drivetrain estimator can fuse both accepted camera updates, but the rest
   * of the robot still benefits from one clearly labeled selected pose for
   * dashboards and aiming debug readouts.
   */
  public void SetPoseEstimateForDrive() {
    poseUpdateAvailable = false;
    poseCamEstimate = null;
    poseUpdateXYTrustFactor = 0.0;
    poseUpdateRotTrustFactor = LARGE_ROTATION_STD_DEV;

    CameraMeasurementDecision selectedDecision = chooseSelectedDecision();

    if (selectedDecision != null) {
      poseCamEstimate = selectedDecision.selectedEstimate;
      poseUpdateXYTrustFactor = selectedDecision.xyStdDev;
      poseUpdateRotTrustFactor = selectedDecision.thetaStdDev;
      poseUpdateAvailable = true;
      selectedCameraName = selectedDecision.cameraName;
    } else {
      selectedCameraName = "None";
    }
  }

  /**
   * Returns every accepted camera measurement from the current loop.
   *
   * We return both accepted camera updates here so the drivetrain estimator can
   * benefit from each valid Limelight solve instead of throwing one away.
   */
  public List<AcceptedVisionMeasurement> getAcceptedMeasurements() {
    List<AcceptedVisionMeasurement> acceptedMeasurements = new ArrayList<>();

    if (cam1Decision.acceptedMeasurement && cam1Decision.selectedEstimate != null) {
      acceptedMeasurements.add(new AcceptedVisionMeasurement(
          cam1Decision.cameraName,
          cam1Decision.selectedEstimate,
          cam1Decision.xyStdDev,
          cam1Decision.thetaStdDev));
    }

    if (cam2Decision.acceptedMeasurement && cam2Decision.selectedEstimate != null) {
      acceptedMeasurements.add(new AcceptedVisionMeasurement(
          cam2Decision.cameraName,
          cam2Decision.selectedEstimate,
          cam2Decision.xyStdDev,
          cam2Decision.thetaStdDev));
    }

    return acceptedMeasurements;
  }

  /**
   * Marks the currently selected vision correction as consumed.
   *
   * The drivetrain calls this after it fuses the accepted measurements so we do
   * not accidentally apply the same camera frames again on a later loop.
   */
  public void UpdateVisionCorrectionAdded() {
    poseUpdateAvailableCam1 = false;
    poseUpdateAvailableCam2 = false;
  }

  /**
   * Stores the drivetrain pose and motion state used to validate vision updates.
   *
   * MegaTag2 expects fresh robot orientation information, and our reliability
   * checks also compare incoming vision poses against the drivetrain's current
   * estimate. Passing the full drive state in here once per loop lets us reject
   * late or unreasonable camera frames more safely.
   */
  public void CollectDriveState(Pose2d drivePose, double driveYawRate, double driveLinearSpeedMetersPerSecond) {
    currentDrivePose = drivePose;
    driveStateAvailable = true;
    currentDriveTheta = drivePose.getRotation().getDegrees();
    currentDriveYawRate = driveYawRate;
    currentDriveLinearSpeedMetersPerSecond = driveLinearSpeedMetersPerSecond;
  }

  /**
   * Pushes the latest drivetrain orientation into both Limelights.
   *
   * MegaTag2 uses the robot's heading and turn rate as an input, so we update both
   * cameras before asking them for new pose estimates.
   */
  private void updateMegaTag2Orientation() {
    LimelightHelpers.SetRobotOrientation(CAMERA_RIGHT, currentDriveTheta, currentDriveYawRate, 0, 0, 0, 0);
    LimelightHelpers.SetIMUMode(CAMERA_RIGHT, 0);
    LimelightHelpers.SetRobotOrientation(CAMERA_LEFT, currentDriveTheta, currentDriveYawRate, 0, 0, 0, 0);
    LimelightHelpers.SetIMUMode(CAMERA_LEFT, 0);
  }

  /**
   * Reads the camera's MegaTag2 estimate and decides whether this loop produced a
   * usable new measurement.
   *
   * This branch intentionally keeps the logic simple: use one pose mode, reject
   * stale or obviously weak frames, and forward good measurements to the
   * drivetrain estimator.
   */
  private CameraMeasurementDecision evaluateCameraMeasurement(String cameraName, double previousTimestamp) {
    CameraMeasurementDecision decision = new CameraMeasurementDecision();
    decision.cameraName = cameraName;

    try {
      PoseEstimate estimate = LimelightHelpers.getBotPoseEstimate_wpiBlue_MegaTag2(cameraName);

      if (estimate == null) {
        decision.statusMessage = "No pose data available from the camera";
        return decision;
      }

      updateDecisionMetrics(decision, estimate);

      if (estimate.tagCount < 1) {
        fillRejectedFrameDetails(decision, estimate);
        decision.statusMessage = "Camera sees no AprilTags";
        return decision;
      }

      if (estimate.timestampSeconds <= previousTimestamp) {
        fillRejectedFrameDetails(decision, estimate);
        decision.statusMessage = "Camera frame is stale and was already processed";
        return decision;
      }

      decision.processedFreshFrame = true;

      if (estimate.latency > MAX_LATENCY_MILLISECONDS) {
        fillRejectedFrameDetails(decision, estimate);
        decision.statusMessage = "Rejected fresh frame because latency was too high";
        return decision;
      }

      if (decision.measurementAgeSeconds > MAX_MEASUREMENT_AGE_SECONDS) {
        fillRejectedFrameDetails(decision, estimate);
        decision.statusMessage = "Rejected fresh frame because it was too old";
        return decision;
      }

      if (!passesBasicReliabilityChecks(estimate)) {
        fillRejectedFrameDetails(decision, estimate);
        decision.statusMessage = "Rejected fresh frame because tag geometry was too weak";
        return decision;
      }

      if (!passesDriveStateValidation(estimate, decision)) {
        return decision;
      }

      fillDecisionFromEstimate(decision, estimate);
      decision.statusMessage = "Accepted fresh MegaTag2 frame";
    } catch (Exception e) {
      decision.statusMessage = "Failed to read Limelight data: " + e.getMessage();
      DriverStation.reportError("Vision camera data not present for " + cameraName + ": " + e.getMessage(),
          e.getStackTrace());
    }

    return decision;
  }

  /**
   * Chooses which accepted camera update to publish as the selected dashboard
   * pose.
   *
   * We keep this simple too: use the newest accepted measurement, with a right
   * camera tie-break if both timestamps are equal.
   */
  private CameraMeasurementDecision chooseSelectedDecision() {
    if (!poseUpdateAvailableCam1 && !poseUpdateAvailableCam2) {
      return null;
    }

    if (poseUpdateAvailableCam1 && !poseUpdateAvailableCam2) {
      return cam1Decision;
    }

    if (!poseUpdateAvailableCam1 && poseUpdateAvailableCam2) {
      return cam2Decision;
    }

    if (cam1Decision.timestampSeconds >= cam2Decision.timestampSeconds) {
      return cam1Decision;
    }

    return cam2Decision;
  }

  /**
   * Applies a short list of tag-quality checks before a frame can be fused.
   *
   * The goal is to keep the rules understandable instead of dynamically blending
   * between several different vision modes and trust calculations.
   */
  private boolean passesBasicReliabilityChecks(PoseEstimate estimate) {
    if (estimate == null) {
      return false;
    }

    if (estimate.tagCount < 1) {
      return false;
    }

    if (estimate.avgTagArea < MIN_TAG_AREA) {
      return false;
    }

    if (estimate.avgTagDist > MAX_TAG_DISTANCE_METERS) {
      return false;
    }

    double maxAmbiguity = getMaxAmbiguity(estimate);
    if (maxAmbiguity > MAX_AMBIGUITY) {
      return false;
    }

    // Weak one-tag solves are still where simple pose pipelines get in trouble
    // most often, so we keep this one stronger special case.
    if (estimate.tagCount == 1) {
      boolean singleTagIsStrong = estimate.avgTagArea >= MIN_SINGLE_TAG_AREA
          && estimate.avgTagDist <= MAX_SINGLE_TAG_DISTANCE_METERS
          && maxAmbiguity <= MAX_SINGLE_TAG_AMBIGUITY;
      return singleTagIsStrong;
    }

    return true;
  }

  /**
   * Copies an accepted pose estimate into the camera decision object.
   *
   * This method stores the selected pose and assigns the branch's fixed trust
   * values.
   */
  private void fillDecisionFromEstimate(CameraMeasurementDecision decision, PoseEstimate estimate) {
    updateDecisionMetrics(decision, estimate);
    decision.acceptedMeasurement = true;
    decision.usingMegaTag1 = false;
    decision.usingMegaTag2 = true;
    decision.selectedEstimate = estimate;
    decision.tagCount = estimate.tagCount;
    decision.avgTagArea = estimate.avgTagArea;
    decision.avgTagDist = estimate.avgTagDist;
    decision.tagSpan = estimate.tagSpan;
    decision.maxAmbiguity = getMaxAmbiguity(estimate);
    decision.xyStdDev = FIXED_XY_STD_DEV;
    decision.thetaStdDev = FIXED_THETA_STD_DEV;
    decision.qualityScore = calculateSimpleQualityScore(estimate);
  }

  /**
   * Stores basic details for a rejected frame.
   *
   * We still publish these values so students can see what the camera observed and
   * understand why the robot ignored that frame.
   */
  private void fillRejectedFrameDetails(CameraMeasurementDecision decision, PoseEstimate estimate) {
    updateDecisionMetrics(decision, estimate);
    decision.selectedEstimate = estimate;
    decision.qualityScore = calculateSimpleQualityScore(estimate);
  }

  /**
   * Updates the diagnostic values we publish for a camera frame.
   *
   * This includes what the camera saw plus how far that pose is from the
   * drivetrain's current estimate. Those extra values make it much easier to tune
   * rejection thresholds on the practice field.
   */
  private void updateDecisionMetrics(CameraMeasurementDecision decision, PoseEstimate estimate) {
    decision.selectedEstimate = estimate;
    decision.timestampSeconds = estimate.timestampSeconds;
    decision.measurementAgeSeconds = Math.max(0.0, Timer.getFPGATimestamp() - estimate.timestampSeconds);
    decision.tagCount = estimate.tagCount;
    decision.avgTagArea = estimate.avgTagArea;
    decision.avgTagDist = estimate.avgTagDist;
    decision.tagSpan = estimate.tagSpan;
    decision.maxAmbiguity = getMaxAmbiguity(estimate);

    if (driveStateAvailable) {
      decision.translationErrorMeters = estimate.pose.getTranslation().getDistance(currentDrivePose.getTranslation());
      decision.headingErrorDegrees = Math.abs(
          estimate.pose.getRotation().minus(currentDrivePose.getRotation()).getDegrees());
    }
  }

  /**
   * Checks whether a camera frame is reasonably close to the drivetrain estimate.
   *
   * This protects the robot from one bad AprilTag solve causing a huge pose jump.
   * This branch only validates translation. Heading is still left to the gyro.
   */
  private boolean passesDriveStateValidation(PoseEstimate estimate, CameraMeasurementDecision decision) {
    updateDecisionMetrics(decision, estimate);

    if (!driveStateAvailable) {
      return true;
    }

    if (decision.translationErrorMeters > MAX_TRANSLATION_JUMP_METERS) {
      decision.statusMessage = "Rejected fresh frame because pose jump was too large";
      return false;
    }

    return true;
  }

  /**
   * Produces a simple debug score for dashboard comparison.
   *
   * This is not used for measurement selection anymore; it is only a compact hint
   * for how strong the current frame looks while tuning.
   */
  private double calculateSimpleQualityScore(PoseEstimate estimate) {
    double score = 0.0;
    double measurementAgeSeconds = Math.max(0.0, Timer.getFPGATimestamp() - estimate.timestampSeconds);

    score += estimate.tagCount * 3.0;
    score += Math.min(estimate.avgTagArea, 1.0) * 4.0;
    score += Math.max(0.0, 4.0 - estimate.avgTagDist);
    score -= getMaxAmbiguity(estimate) * 5.0;
    score -= measurementAgeSeconds * 10.0;

    return score;
  }

  /**
   * Finds the worst ambiguity value in a pose estimate's tag list.
   *
   * Ambiguity tells us how unsure the camera is about a tag solution, so the worst
   * ambiguity in the frame is a useful quick summary for trust decisions.
   */
  private double getMaxAmbiguity(PoseEstimate estimate) {
    if (estimate == null || estimate.rawFiducials == null || estimate.rawFiducials.length == 0) {
      return 0.0;
    }

    double maxAmbiguity = 0.0;

    for (RawFiducial fiducial : estimate.rawFiducials) {
      if (fiducial.ambiguity > maxAmbiguity) {
        maxAmbiguity = fiducial.ambiguity;
      }
    }

    return maxAmbiguity;
  }

  /**
   * Publishes detailed vision debugging information to SmartDashboard.
   *
   * The goal is to make the robot's decisions understandable: which camera saw a
   * frame, which pose mode was chosen, what the tag quality looked like, and why a
   * frame was accepted or rejected.
   */
  private void updateShuffleboard() {
    publishCameraTelemetry("Vision/Right", cam1Decision);
    publishCameraTelemetry("Vision/Left", cam2Decision);

    SmartDashboard.putBoolean("Vision/Selected/Available", poseUpdateAvailable);
    SmartDashboard.putString("Vision/Selected/Camera", getSelectedCameraLabel());
    SmartDashboard.putString("Vision/Selected/PreferredCamera", selectedCameraName);
    SmartDashboard.putString("Vision/Config/PoseMode", "MegaTag2Only");
    SmartDashboard.putNumber("Vision/Selected/XYStdDev", poseUpdateXYTrustFactor);
    SmartDashboard.putNumber("Vision/Selected/ThetaStdDev", poseUpdateRotTrustFactor);
    publishSelectedPoseTelemetry();
    SmartDashboard.putNumber("Vision/Drive/YawRateDegPerSec", currentDriveYawRate);
    SmartDashboard.putNumber("Vision/Drive/LinearSpeedMetersPerSecond", currentDriveLinearSpeedMetersPerSecond);

    SmartDashboard.putBoolean("Vision/Right/UsingMT1", usingCam1MT1);
    SmartDashboard.putBoolean("Vision/Right/UsingMT2", usingCam1MT2);
    SmartDashboard.putBoolean("Vision/Left/UsingMT1", usingCam2MT1);
    SmartDashboard.putBoolean("Vision/Left/UsingMT2", usingCam2MT2);
  }

  /**
   * Publishes the currently selected vision pose as explicit X/Y/heading numbers.
   *
   * Having these values next to the drivetrain's fused pose makes it much easier
   * to tell whether a bad field position is coming from the raw Limelight solve or
   * from how the estimator is blending that solve with odometry and gyro data.
   */
  private void publishSelectedPoseTelemetry() {
    if (poseCamEstimate == null) {
      SmartDashboard.putNumber("Vision/Selected/PoseX", Double.NaN);
      SmartDashboard.putNumber("Vision/Selected/PoseY", Double.NaN);
      SmartDashboard.putNumber("Vision/Selected/PoseAngle", Double.NaN);
      return;
    }

    SmartDashboard.putNumber("Vision/Selected/PoseX", poseCamEstimate.pose.getX());
    SmartDashboard.putNumber("Vision/Selected/PoseY", poseCamEstimate.pose.getY());
    SmartDashboard.putNumber("Vision/Selected/PoseAngle", poseCamEstimate.pose.getRotation().getDegrees());
  }

  /**
   * Publishes one camera's decision details under a shared dashboard prefix.
   *
   * Using separate left and right keys prevents one camera from overwriting the
   * other camera's telemetry while we tune the system.
   */
  private void publishCameraTelemetry(String dashboardPrefix, CameraMeasurementDecision decision) {
    SmartDashboard.putBoolean(dashboardPrefix + "/FreshFrame", decision.processedFreshFrame);
    SmartDashboard.putBoolean(dashboardPrefix + "/Accepted", decision.acceptedMeasurement);
    SmartDashboard.putString(dashboardPrefix + "/Mode", getModeLabel(decision));
    SmartDashboard.putString(dashboardPrefix + "/Status", decision.statusMessage);
    SmartDashboard.putNumber(dashboardPrefix + "/Timestamp", decision.timestampSeconds);
    SmartDashboard.putNumber(dashboardPrefix + "/MeasurementAgeSeconds", decision.measurementAgeSeconds);
    SmartDashboard.putNumber(dashboardPrefix + "/TagCount", decision.tagCount);
    SmartDashboard.putNumber(dashboardPrefix + "/AvgTagArea", decision.avgTagArea);
    SmartDashboard.putNumber(dashboardPrefix + "/AvgTagDist", decision.avgTagDist);
    SmartDashboard.putNumber(dashboardPrefix + "/TagSpan", decision.tagSpan);
    SmartDashboard.putNumber(dashboardPrefix + "/MaxAmbiguity", decision.maxAmbiguity);
    SmartDashboard.putNumber(dashboardPrefix + "/TranslationErrorMeters", decision.translationErrorMeters);
    SmartDashboard.putNumber(dashboardPrefix + "/HeadingErrorDegrees", decision.headingErrorDegrees);
    SmartDashboard.putNumber(dashboardPrefix + "/QualityScore", decision.qualityScore);
    SmartDashboard.putNumber(dashboardPrefix + "/XYStdDev", decision.xyStdDev);
    SmartDashboard.putNumber(dashboardPrefix + "/ThetaStdDev", decision.thetaStdDev);
    publishDecisionPoseTelemetry(dashboardPrefix, decision);
  }

  /**
   * Publishes the raw pose stored in one camera decision.
   *
   * These numbers let us compare the left and right Limelight solves directly. If
   * one camera is consistently wrong while the other looks reasonable, that usually
   * points to a camera-specific robot-space configuration issue.
   */
  private void publishDecisionPoseTelemetry(String dashboardPrefix, CameraMeasurementDecision decision) {
    if (decision.selectedEstimate == null) {
      SmartDashboard.putNumber(dashboardPrefix + "/PoseX", Double.NaN);
      SmartDashboard.putNumber(dashboardPrefix + "/PoseY", Double.NaN);
      SmartDashboard.putNumber(dashboardPrefix + "/PoseAngle", Double.NaN);
      return;
    }

    SmartDashboard.putNumber(dashboardPrefix + "/PoseX", decision.selectedEstimate.pose.getX());
    SmartDashboard.putNumber(dashboardPrefix + "/PoseY", decision.selectedEstimate.pose.getY());
    SmartDashboard.putNumber(dashboardPrefix + "/PoseAngle",
        decision.selectedEstimate.pose.getRotation().getDegrees());
  }

  /**
   * Returns a short label describing which camera measurement was chosen.
   *
   * This helps students quickly see whether the drivetrain is currently trusting the
   * right camera, the left camera, or no vision update at all.
   */
  private String getSelectedCameraLabel() {
    if (!poseUpdateAvailable || poseCamEstimate == null) {
      return "None";
    }

    if (CAMERA_RIGHT.equals(selectedCameraName)) {
      return "Right";
    }

    if (CAMERA_LEFT.equals(selectedCameraName)) {
      return "Left";
    }

    return "Unknown";
  }

  public String getSelectedCameraName() {
    return getSelectedCameraLabel();
  }

  /**
   * Returns a short label describing which Limelight pose mode was used.
   *
   * We keep this text simple so the dashboard is easy to read during a match or a
   * practice session.
   */
  private String getModeLabel(CameraMeasurementDecision decision) {
    if (decision.usingMegaTag1) {
      return "MegaTag1";
    }

    if (decision.usingMegaTag2) {
      return "MegaTag2";
    }

    return "None";
  }

  @Override
  public void periodic() {
    // This method runs every scheduler loop and decides whether a brand new
    // Limelight frame should be forwarded to the drivetrain's pose estimator.
    if (poseUpdatesFromCameraActive) {
      updateMegaTag2Orientation();
      SetPoseEstimateInfoCam1();
      SetPoseEstimateInfoCam2();
      SetPoseEstimateForDrive();
    } else {
      poseUpdateAvailable = false;
      poseUpdateAvailableCam1 = false;
      poseUpdateAvailableCam2 = false;
      usingCam1MT1 = false;
      usingCam1MT2 = false;
      usingCam2MT1 = false;
      usingCam2MT2 = false;
      poseCamEstimate = null;
      selectedCameraName = "None";
    }

    updateShuffleboard();
  }
}
