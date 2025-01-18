package frc.robot.util;

import edu.wpi.first.math.estimator.SwerveDrivePoseEstimator;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.kinematics.SwerveDriveKinematics;
import edu.wpi.first.math.kinematics.SwerveModulePosition;

// extended version of the pose estimator that the drivetrain uses. For now this doesn't do anthing
// special but later if we wanted to upgrade the pose system we would just need to add functionality
// to this one
public class VortechsPoseEstimator extends SwerveDrivePoseEstimator {
  public VortechsPoseEstimator(
      SwerveDriveKinematics kinematics,
      Rotation2d gyroAngle,
      SwerveModulePosition[] modulePositions,
      Pose2d initialPoseMeters) {
    super(kinematics, gyroAngle, modulePositions, initialPoseMeters);
  }

  
}
