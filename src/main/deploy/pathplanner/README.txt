Place PathPlanner files here for Elastic auton preview.

- Autos go in `src/main/deploy/pathplanner/autos/*.auto`
- Paths go in `src/main/deploy/pathplanner/paths/*.path`

When running robot simulation, connect Elastic to `localhost` and add:
- A Field widget for `SmartDashboard/Elastic/Auton Field`
- A Field widget for `SmartDashboard/Elastic/Robot Pose Field`
- A chooser widget for `SmartDashboard/Elastic/Auto Chooser`
