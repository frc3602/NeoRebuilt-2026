package frc.robot.telemetry;

import com.pathplanner.lib.auto.AutoBuilder;
import com.pathplanner.lib.commands.PathPlannerAuto;
import com.pathplanner.lib.path.PathPlannerPath;
import com.pathplanner.lib.util.FlippingUtil;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.networktables.NetworkTable;
import edu.wpi.first.networktables.NetworkTableInstance;
import edu.wpi.first.networktables.StringPublisher;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.smartdashboard.Field2d;
import edu.wpi.first.wpilibj.smartdashboard.SendableChooser;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import frc.robot.subsystems.Drivetrain;
import java.util.ArrayList;
import java.util.Collections;
import java.util.List;

public class AutonFieldPreview {
  private static final String kFieldWidgetName = "Elastic/Auton Field";
  private static final String kChooserWidgetName = "Elastic/Auto Chooser";

  private final Field2d m_field = new Field2d();
  private final List<String> m_previewObjectNames = new ArrayList<>();
  private final StringPublisher m_selectedAutoPublisher;
  private final StringPublisher m_previewStatusPublisher;

  private String m_lastAutoName = null;
  private boolean m_lastShouldFlip = false;
  private Pose2d m_previewStartingPose = null;

  public AutonFieldPreview(SendableChooser<String> autoChooser) {
    SmartDashboard.putData(kFieldWidgetName, m_field);
    SmartDashboard.putData(kChooserWidgetName, autoChooser);

    NetworkTable autoTable = NetworkTableInstance.getDefault().getTable("Elastic").getSubTable("Auto");
    m_selectedAutoPublisher = autoTable.getStringTopic("SelectedAuto").publish();
    m_previewStatusPublisher = autoTable.getStringTopic("PreviewStatus").publish();
    m_previewStatusPublisher.set("Waiting for auto selection");
  }

  public void update(Drivetrain drivetrain, String selectedAutoName) {
    String normalizedAutoName = selectedAutoName == null ? "" : selectedAutoName;
    boolean shouldFlip = shouldFlipPreview();
    m_selectedAutoPublisher.set(normalizedAutoName.isBlank() ? "None" : normalizedAutoName);

    if (!normalizedAutoName.equals(m_lastAutoName) || shouldFlip != m_lastShouldFlip) {
      loadAutoPreview(normalizedAutoName, shouldFlip);
      m_lastAutoName = normalizedAutoName;
      m_lastShouldFlip = shouldFlip;
    }

    Pose2d displayedPose = drivetrain.getEstimatedPose();
    if (DriverStation.isDisabled() && m_previewStartingPose != null) {
      displayedPose = m_previewStartingPose;
    }

    m_field.setRobotPose(displayedPose);
  }

  private void loadAutoPreview(String autoName, boolean shouldFlip) {
    clearPreviewObjects();

    if (autoName.isBlank()) {
      m_previewStartingPose = null;
      m_previewStatusPublisher.set("No auto selected");
      return;
    }

    try {
      List<PathPlannerPath> paths = PathPlannerAuto.getPathGroupFromAutoFile(autoName);
      if (paths.isEmpty()) {
        m_previewStatusPublisher.set("Auto has no paths");
        return;
      }

      for (int i = 0; i < paths.size(); i++) {
        PathPlannerPath displayPath = shouldFlip ? paths.get(i).flipPath() : paths.get(i);
        List<Pose2d> poses = displayPath.getPathPoses();
        String objectName = "Auto Path " + i;
        m_field.getObject(objectName).setPoses(poses);
        m_previewObjectNames.add(objectName);
      }

      m_previewStartingPose = getStartingPose(autoName, paths, shouldFlip);
      if (m_previewStartingPose != null) {
        m_field.getObject("Auto Start").setPose(m_previewStartingPose);
        m_previewObjectNames.add("Auto Start");
      }

      m_previewStatusPublisher.set("Loaded preview for " + autoName);
    } catch (Exception ex) {
      clearPreviewObjects();
      DriverStation.reportWarning("Failed to load auto preview for " + autoName, false);
      m_previewStatusPublisher.set("Failed to load preview for " + autoName);
    }
  }

  private void clearPreviewObjects() {
    for (String objectName : m_previewObjectNames) {
      m_field.getObject(objectName).setPoses(Collections.emptyList());
    }
    m_field.getObject("Auto Start").setPoses(Collections.emptyList());
    m_previewObjectNames.clear();
    m_previewStartingPose = null;
  }

  private Pose2d getStartingPose(String autoName, List<PathPlannerPath> paths, boolean shouldFlip) {
    Pose2d startingPose;
    if (AutoBuilder.isConfigured()) {
      startingPose = new PathPlannerAuto(autoName).getStartingPose();
    } else {
      if (paths.isEmpty()) {
        return null;
      }

      PathPlannerPath firstPath = paths.get(0);
      Rotation2d rotation = Rotation2d.kZero;
      if (firstPath.getIdealStartingState() != null) {
        rotation = firstPath.getIdealStartingState().rotation();
      }

      startingPose = new Pose2d(firstPath.getPoint(0).position, rotation);
    }

    if (startingPose == null) {
      return null;
    }

    return shouldFlip ? FlippingUtil.flipFieldPose(startingPose) : startingPose;
  }

  private boolean shouldFlipPreview() {
    return DriverStation.getAlliance().orElse(DriverStation.Alliance.Blue)
        == DriverStation.Alliance.Red;
  }
}
