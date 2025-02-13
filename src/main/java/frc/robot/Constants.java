// Copyright 2021-2025 FRC 6328
// http://github.com/Mechanical-Advantage
//
// This program is free software; you can redistribute it and/or
// modify it under the terms of the GNU General Public License
// version 3 as published by the Free Software Foundation or
// available in the root directory of this project.
//
// This program is distributed in the hope that it will be useful,
// but WITHOUT ANY WARRANTY; without even the implied warranty of
// MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE. See the
// GNU General Public License for more details.

package frc.robot;

import edu.wpi.first.math.geometry.Rotation3d;
import edu.wpi.first.math.geometry.Transform3d;
import edu.wpi.first.math.geometry.Translation3d;
import edu.wpi.first.wpilibj.RobotBase;

/**
 * This class defines the runtime mode used by AdvantageKit. The mode is always "real" when running
 * on a roboRIO. Change the value of "simMode" to switch between "sim" (physics sim) and "replay"
 * (log replay from a file).
 */
public final class Constants {
  public static final Mode simMode = Mode.SIM;
  public static final Mode currentMode = RobotBase.isReal() ? Mode.REAL : simMode;

  public static enum Mode {
    /** Running on a real robot. */
    REAL,

    /** Running a physics simulator. */
    SIM,

    /** Replaying from a log file. */
    REPLAY
  }

  // constants used for the elevator
  public static final int ELEVATOR_MOTOR_LEFT_ID = 21;
  public static final int ELEVATOR_MOTOR_RIGHT_ID = 22;
  public static final String ELEVATOR_CANBUS = "rio";

  public static class VisionConstants {
    // placeholder translation to use in the estimator(? i'm not sure how it'll detect the camera)
    public static final Transform3d robotToCam =
        new Transform3d(
            new Translation3d(0.5, 0.0, 0.5),
            new Rotation3d(
                0, 0,
                0)); // Cam mounted facing forward, half a meter forward of center, half a meter up
    // from center.
    // placeholder translation to use in the estimator(? i'm not sure how it'll detect the camera)
    public static final Transform3d camToRobot =
        new Transform3d(
            new Translation3d(-0.5, -0.0, -0.5),
            new Rotation3d(
                0, 0,
                0)); // Cam mounted facing forward, half a meter forward of center, half a meter up
    // from center.
  }

  // public static class Vision {
  //   public static final String kCameraName = "Arducam_1";
  //   // Cam mounted facing forward, half a meter forward of center, half a meter up from center.
  //   // !!! these are placeholder numbers from the example
  //   public static final Transform3d kRobotToCam =
  //       new Transform3d(new Translation3d(0.5, 0.0, 0.5), new Rotation3d(0, 0, 0));

  //   // The layout of the AprilTags on the field
  //   public static final AprilTagFieldLayout kTagLayout =
  //       AprilTagFields.kDefaultField.loadAprilTagLayoutField();

  //   // The standard deviations of our vision estimated poses, which affect correction rate
  //   // (Fake values. Experiment and determine estimation noise on an actual robot.)
  //   public static final Matrix<N3, N1> kSingleTagStdDevs = VecBuilder.fill(4, 4, 8);
  //   public static final Matrix<N3, N1> kMultiTagStdDevs = VecBuilder.fill(0.5, 0.5, 1);
  // }
}
