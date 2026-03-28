# Field Tuning Guide

This guide is for quick on-field reference.

It focuses on the tuning values that are most likely to be adjusted during practice, calibration, and event troubleshooting.

All game-piece references use the team term `fuel`.

Line numbers match the current code on this branch and may need to be re-checked after future refactors.

## How To Use This Guide

1. Start in the `Start Here` section before touching the advanced items.
2. Change one value at a time.
3. Record the old value before changing it.
4. Re-test the exact symptom that caused the change.

## Start Here

| Area | What To Tune | File | Line | Symptom -> Likely Fix |
| --- | --- | --- | --- | --- |
| Driver feel | `MaxSpeed`, `MaxAngularRate`, drive deadbands | `src/main/java/frc/robot/RobotContainer.java` | 43, 44, 47, 48 | Robot feels twitchy or drifts on small stick input -> lower speed or turn caps, or raise deadbands. Robot feels too slow -> do the opposite. |
| Shot power and shot map | `kOverallShotVelocityScale`, shooter target model, reference shot distance, launch angle | `src/main/java/frc/robot/Constants.java` | 26, 43, 66, 67, 71 | Tracked shots are short from most spots -> raise overall shot scale. Tracked shots are long -> lower it. Miss changes with distance -> re-check the reference distance or launch-angle model instead of only scaling speed. |
| Shooter response | Shooter `kS`, `kV`, `kP`, Motion Magic acceleration and jerk | `src/main/java/frc/robot/subsystems/Shooter.java` | 154, 155, 157, 161, 163 | Spin-up is slow or the shooter never quite reaches speed -> increase `kV` or `kS` slightly, or make Motion Magic more aggressive. Overshoot or hunting -> reduce `kP`, acceleration, or jerk. |
| Feed timing for shots | Shooter ready tolerance and tracked-shot feed delay | `src/main/java/frc/robot/SuperStructure.java` | 16, 17 | Fuel feeds before the shooter is truly stable -> tighten tolerance or add delay. Robot waits too long to fire -> loosen tolerance or reduce delay. |
| Turret tracking | Turret `kP`, Motion Magic profile, position tolerance, translational lead, rotational lead, center-field switch window | `src/main/java/frc/robot/Constants.java` | 218, 222, 225, 228, 229, 230, 231 | Shots miss behind while driving -> increase lead gain. Shots miss ahead -> decrease it. Turret oscillates -> reduce `kP` or motion-profile aggressiveness. Turret says ready too early -> tighten turret tolerance. |
| Vision acceptance | Tag area, distance, ambiguity, latency, pose jump limits | `src/main/java/frc/robot/subsystems/Limelight_Pose.java` | 81, 87, 89, 470, 496, 596 | Robot rarely accepts tags -> relax min area, max distance, ambiguity, or latency. Pose jumps or snaps badly -> tighten jump, ambiguity, or latency checks. |
| Vision trust weighting | `XYStdDev`, `ThetaStdDev`, quality weighting | `src/main/java/frc/robot/subsystems/Limelight_Pose.java` | 620, 693, 736 | Vision corrections feel too weak and aim takes too long to settle after stopping -> lower trust baselines or increase stationary bonuses. Pose chatters or overreacts -> raise them. |
| Auton tracking | PathPlanner translation and rotation P gains, robot model, default constraints | `src/main/java/frc/robot/subsystems/Drivetrain.java` and `src/main/deploy/pathplanner/settings.json` | 81, 82 and 7, 12, 17 | Autos overshoot or oscillate -> reduce PathPlanner P gains. Autos lag, cut corners, or feel globally wrong -> fix the PathPlanner robot model and default constraints. |

## Advanced / Less Often

| Area | What To Tune | File | Line | Symptom -> Likely Fix |
| --- | --- | --- | --- | --- |
| Feed path speeds | Spindexer and receiver ratios, feed PID, feed acceleration | `src/main/java/frc/robot/Constants.java` | 241, 242, 252, 258 | Fuel stalls between receiver, spindexer, and shooter -> speed up the slow section or raise feed `kP` or `kV`. Fuel is shoved through too hard or double-feeds -> back the ratios or acceleration down. |
| Intake roller | Intake forward and reverse speeds, intake PID, intake acceleration | `src/main/java/frc/robot/Constants.java` | 183, 185, 191 | Intake does not pull fuel in cleanly -> increase forward speed or acceleration. Intake spits fuel back out or is too violent -> reduce them. |
| Pivot deploy and stow | Pivot current limit, raise and drop angles, pivot PID and gravity feedforward, Motion Magic profile | `src/main/java/frc/robot/Constants.java` | 195, 198, 199, 200, 203, 204 | Intake does not reach full up or down -> fix the angle setpoints. Pivot sags, stalls, or moves too violently -> tune current limit, `kP`, `kG`, and the motion profile. |
| Climber motion | Climber current limit, PID, Motion Magic profile, up and down presets, tolerance | `src/main/java/frc/robot/Constants.java` | 158, 159, 163, 166, 167, 168 | Climber stops short, bangs the ends, or chatters at target -> adjust end positions, tolerance, and motion profile before touching gains. |
| Swerve behavior | Steer gains, drive gains, slip current, top speed, module encoder offsets | `src/main/java/frc/robot/generated/TunerConstants.java` | 30, 36, 58, 81, 140, 151, 162, 173 | Modules oscillate or feel sloppy -> retune steer and drive gains. Robot crabs when commanded straight -> recalibrate encoder offsets first. |
| Field target geometry | Tower positions, pass-corner positions, turret preset angles | `src/main/java/frc/robot/Constants.java` and `src/main/java/frc/robot/subsystems/Turret.java` | 274, 275, 278, 279 and 85, 89, 93, 97 | Tracked shots are consistently angularly off even with good pose -> verify field target coordinates. Only preset shots miss -> tune the preset angles instead. |

## Known Gotchas

- Shooter tuning is split in the software: `Constants.ShooterConstants.kShooterVelocity*` exists, but the live shooter gains are currently hardcoded in `src/main/java/frc/robot/subsystems/Shooter.java`.
- Failsafe-shot tuning is split: `SuperStructure` currently points failsafe at the normal shooter target, while `Constants.ShooterConstants.kShooterFailsafeSpeed` exists but is not used.
- Tracked-shot feed timing is also split: the full tracked-shot ready check includes turret and shooter, but the feed gate in `SuperStructure` currently waits only on shooter readiness plus delay.
- Auton robot-model tuning is split: `Drivetrain` loads PathPlanner GUI settings first, so `src/main/deploy/pathplanner/settings.json` is the active source unless that load fails.
- `Constants.TurretConstants.kTurretProjectileSpeedMetersPerSecond` and `Constants.TurretConstants.kTurretMinimumLookaheadSeconds` appear to be unused today, so they are not included above.
