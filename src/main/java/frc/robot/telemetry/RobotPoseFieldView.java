package frc.robot.telemetry;

import edu.wpi.first.wpilibj.smartdashboard.Field2d;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import frc.robot.subsystems.Drivetrain;

public class RobotPoseFieldView {
  private static final String kFieldWidgetName = "Elastic/Robot Pose Field";

  private final Field2d m_field = new Field2d();

  public RobotPoseFieldView() {
    SmartDashboard.putData(kFieldWidgetName, m_field);
  }

  public void update(Drivetrain drivetrain) {
    m_field.setRobotPose(drivetrain.getEstimatedPose());
  }
}
