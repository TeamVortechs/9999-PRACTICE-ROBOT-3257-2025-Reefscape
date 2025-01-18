package frc.robot.util;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Pose3d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Transform2d;
import java.util.HashMap;

// using this mainly for april tag poses for limelight
// this is not updated at all and will not work
public class FieldPositions {
  // pretty sure setting the initial capacity helps with performance
  private static HashMap<Integer, Pose3d> aprilTagMap = new HashMap<>(16);

  // probably don't need this
  public static Pose2d AmpPose = new Pose2d();
  public static Pose2d SpeakerPose = new Pose2d();

  public static Pose3d getTagPose(int tagID) {
    if (aprilTagMap.isEmpty()) {
      buildMaps();
    }

    return aprilTagMap.get(tagID);
  }

  public static Pose2d getFieldPose(int tagID, Pose2d tagRelativePose) {
    // not sure if this will work
    return getTagPose(tagID)
        .toPose2d()
        .plus(new Transform2d(tagRelativePose.getTranslation(), tagRelativePose.getRotation()));
  }

  private static void buildMaps() {
    // example:
    aprilTagMap.put(1, new Pose3d());
    // repeat this for every tag
  }

  // this function does not work for values over 180 and probably does not work
  public static Rotation2d poseToPoseRotation(Pose2d basePose, Pose2d rotatorPose2d) {
    double xDiff = basePose.getX() - rotatorPose2d.getX();
    double yDiff = basePose.getY() - rotatorPose2d.getY();

    return new Rotation2d(Math.atan(xDiff / yDiff));
  }

  // gets the id of the teams home speaker
  // public static int getHomeSpeakerID() {
  //     if(DriverStation.getAlliance().get() == Alliance.Blue) {
  //         return LimelightTags.BLUE_SPEAKER.getID();
  //     } else {
  //         return LimelightTags.RED_SPEAKER.getID();
  //     }
  // }

  public static double getDistance(Pose2d pose1, Pose2d pose2) {
    return Math.sqrt(
        Math.pow(pose1.getX() - pose2.getX(), 2) + Math.pow(pose1.getY() - pose2.getY(), 2));
  }
}
