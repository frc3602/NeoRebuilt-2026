# Simple Pose Test Checklist

This checklist is for the `simple-pose-test` branch.

Its goal is to help the team simplify the Limelight pose pipeline without losing enough localization quality to hurt driving, aiming, or autonomous.

Use this document before coding, during implementation, and on the practice field.

## Goal

Win condition:

- Keep a simpler two-Limelight pose system only if tracked-shot consistency, pose stability, and autonomous repeatability are not meaningfully worse than the current branch.

Main question:

- Is the custom pose-selection logic in `Limelight_Pose` buying enough real robot performance to justify its added complexity?

## Branch Scope

Target branch:

- `simple-pose-test`

Recommended simplified design:

- Keep both Limelights.
- Keep one shared `Limelight_Pose` subsystem.
- Keep the drivetrain estimator fusion flow.
- Keep a selected vision pose for dashboards.
- Use one pose mode per camera for the whole experiment.
- Give each accepted camera frame fixed trust values instead of dynamic trust weighting.
- Fuse both accepted camera measurements when both are valid.

Recommended cuts:

- Remove MegaTag1 vs MegaTag2 arbitration per camera.
- Remove camera quality scoring and preferred-camera hysteresis.
- Remove most dynamic trust math.
- Remove any tuning logic that is only there to compare pose modes against each other.

Recommended safeguards to keep:

- Fresh timestamp checks.
- Measurement age and latency rejection.
- Minimum tag-count checks.
- Pose-jump sanity checks against drivetrain pose.

## Implementation Checklist

- Confirm the work starts from branch `simple-pose-test`.
- Keep the public shape of `Limelight_Pose` as stable as possible so the rest of the robot needs minimal changes.
- Prefer changing only `src/main/java/frc/robot/subsystems/Limelight_Pose.java` unless a small telemetry cleanup is truly needed.
- Keep `getAcceptedMeasurements()` working so drivetrain fusion in `src/main/java/frc/robot/subsystems/Drivetrain.java` stays familiar.
- Keep the selected-pose outputs used by `src/main/java/frc/robot/telemetry/ElasticTelemetry.java`.
- Choose one pose mode for the experiment and document it in code comments and commit messages.
- Use fixed `XYStdDev` and `ThetaStdDev` values for the first version of the branch.
- Keep enough dashboard output to answer basic field questions:
- Which camera was accepted?
- Did either camera have a fresh frame?
- Why was a frame rejected?
- What pose was selected?
- Avoid adding new tuning knobs until the simplest version has been tested once.
- Do not mix this experiment with unrelated drivetrain, turret, or shooter changes.

## Pre-Deploy Checklist

- Build successfully before deploying.
- Confirm both Limelights are online in NetworkTables.
- Confirm both Limelight names still match the code expectations:
- `limelight-right`
- `limelight-left`
- Confirm field layout and AprilTag map are unchanged from baseline testing.
- Confirm camera mounting, wiring, and pipeline settings have not changed between baseline and simplified-branch tests.
- Confirm odometry reset procedure is the same as baseline.
- Confirm the operator knows this is a pose experiment branch before testing shots or autonomous.

## Baseline Checklist

Complete these on the current reference branch before judging the simplified branch:

- Record stationary pose behavior at several known field spots.
- Record drive-stop-settle behavior after a medium-length drive.
- Record slow driving while tags stay in view.
- Record aggressive turning or zig-zag behavior.
- Record one-camera-visible situations for both the left and right cameras.
- Record a few stationary tracked shots.
- Record a few drive-then-stop tracked shots.
- Record at least one short autonomous routine that depends on pose quality.

For each test, write down:

- Whether pose visibly snaps or drifts.
- How fast pose settles after stopping.
- Whether turret aim appears stable.
- Whether shot outcome feels normal, better, or worse.
- Whether autonomous end pose looks repeatable.

## Field Test Checklist

Run the same tests on `simple-pose-test` in the same order, with the same robot setup, and ideally the same driver.

### Stationary Pose

- Place the robot on a known tape mark.
- Let the robot sit for 2 to 3 seconds.
- Check whether fused pose settles smoothly.
- Compare final pose confidence to the baseline branch.

### Drive Then Stop

- Drive 10 to 20 feet.
- Stop firmly.
- Wait 2 seconds.
- Check whether the pose settles as fast as baseline.
- Watch for delayed corrections or sudden jumps.

### Slow Tracking Drive

- Drive slowly while AprilTags stay in view.
- Watch for pose wobble.
- Watch for aim instability or visible heading weirdness.

### Aggressive Motion

- Turn quickly or zig-zag.
- Watch for bad estimator snaps.
- Check whether the robot feels less predictable than baseline.

### One-Camera Coverage

- Test a spot where the right camera has the better tag view.
- Test a spot where the left camera has the better tag view.
- Test a spot where both cameras can see tags.
- Check whether one weak camera can disturb otherwise good pose estimates.

### Shot Validation

- Test stationary tracked shots.
- Test drive-then-stop tracked shots.
- If the team normally shoots while moving, test that too.
- Compare shot consistency to the baseline branch, not just whether a single shot went in.

### Autonomous Sanity

- Run at least one short, repeatable auto.
- Compare start-to-finish behavior with the baseline branch.
- Watch for path-end drift, bad heading corrections, or setup inconsistency before shooting.

## Elastic Success Signals

Watch these topics in Elastic during the baseline branch and the `simple-pose-test` branch:

### `Elastic/Drive`

Topics:

- `Pose`
- `PoseXMeters`
- `PoseYMeters`
- `HeadingDegrees`

Good signs:

- Drive pose moves smoothly while the robot drives.
- Drive pose settles quickly after the robot stops.
- Heading stays believable during turns and does not suddenly snap.
- End pose after a repeatable path or auto looks similar from run to run.

Warning signs:

- Pose jumps several feet in one update.
- Heading suddenly rotates to an obviously wrong angle.
- Pose keeps drifting after the robot is stationary with tags in view.
- Repeated autos end in noticeably different places.

### `Elastic/Vision`

Topics:

- `HasVisionPose`
- `SelectedCamera`
- `Pose`
- `PoseXMeters`
- `PoseYMeters`
- `HeadingDegrees`
- `XYStdDev`
- `ThetaStdDev`

Good signs:

- `HasVisionPose` goes true reliably when tags are visible.
- `SelectedCamera` changes only when it makes sense from robot position or tag visibility.
- Vision pose is reasonably close to the drive pose most of the time.
- `XYStdDev` and `ThetaStdDev` stay stable for similar viewing conditions on repeated tests.
- When the robot stops with tags in view, the vision pose and drive pose converge instead of fighting each other.

Warning signs:

- `HasVisionPose` flickers on and off constantly in situations where tags should be usable.
- `SelectedCamera` thrashes back and forth without a clear reason.
- Vision pose repeatedly disagrees with drive pose by a large amount.
- `XYStdDev` or `ThetaStdDev` behave inconsistently for the same test setup.
- Vision heading looks obviously wrong compared with the robot's real orientation.

### What To Compare Between Branches

- How often `HasVisionPose` is available at your normal scoring and driving locations.
- Whether `SelectedCamera` behavior feels predictable and explainable.
- Whether the vision pose is calmer after the robot stops.
- Whether the drive pose converges faster after a stop.
- Whether repeated autos produce tighter end-pose grouping.

### Practical Success Definition Using Elastic

Treat Elastic as supporting evidence that the simpler branch is working if most of these are true:

- Vision pose appears when expected and stays present long enough to help.
- Drive pose looks smoother or equally smooth compared with baseline.
- Vision and drive pose disagree less, or at least not more, in normal use.
- Post-stop settling is as fast or faster than baseline.
- Camera selection looks understandable to the operators.

Treat Elastic as evidence against the simpler branch if these show up often:

- Vision availability drops in places where the current branch works.
- Drive pose snaps more often after vision updates.
- Vision and drive pose fight each other during shot setup.
- Camera selection becomes noisy or confusing.
- Auto end poses scatter more than baseline.

## Pass / Fail Criteria

Treat the branch as a success if most of these are true:

- Stationary pose is equally stable or better.
- Pose settles after stopping at least as well as baseline.
- No new large pose snaps are observed.
- Tracked-shot consistency is unchanged or only slightly worse.
- Autonomous repeatability is unchanged or only slightly worse.
- The robot feels simpler and more predictable to drive and aim with.

Treat the branch as a failure if any of these happen repeatedly:

- Pose jitters more while approaching or preparing a shot.
- Field pose visibly teleports or rotates incorrectly.
- One-camera cases produce bad corrections often enough to matter.
- Autonomous routines end in noticeably less repeatable poses.
- Operators feel aim takes longer to settle after the robot stops.

## What Different Outcomes Mean

- If stationary behavior is good but moving behavior gets worse, the old motion-aware trust logic may have been helping.
- If one-camera cases get much worse, the old per-camera selection logic may have been helping.
- If everything feels about the same, the current complexity may not be earning its keep.
- If the simpler branch is more predictable, the current logic may be overcomplicating or overreacting.

## Notes Template

Use a simple log like this during testing:

| Test | Branch | Result | Notes |
| --- | --- | --- | --- |
| Stationary pose at midfield | `main` | Pass | Settled quickly, no jump |
| Stationary pose at midfield | `simple-pose-test` | Pass | Slightly slower settle, still acceptable |
| Drive-stop-settle | `main` | Pass | Stable |
| Drive-stop-settle | `simple-pose-test` | Fail | One visible pose snap after stop |

## References

- `src/main/java/frc/robot/subsystems/Limelight_Pose.java`
- `src/main/java/frc/robot/subsystems/Drivetrain.java`
- `src/main/java/frc/robot/telemetry/ElasticTelemetry.java`
