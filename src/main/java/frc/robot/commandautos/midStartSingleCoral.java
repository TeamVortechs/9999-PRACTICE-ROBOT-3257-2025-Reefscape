package frc.robot.commandautos;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.DriverStation.Alliance;
import edu.wpi.first.wpilibj2.command.InstantCommand;
import edu.wpi.first.wpilibj2.command.PrintCommand;
import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import frc.robot.commands.autoCommands.DriveCommands;
import frc.robot.subsystems.drive.Drive;
import frc.robot.subsystems.elevator.Elevator;
import frc.robot.subsystems.wrist.Wrist;

public class midStartSingleCoral extends SequentialCommandGroup {

  public midStartSingleCoral(Wrist wrist, Elevator elevator, Drive drive) {
    addRequirements(
        wrist, elevator, drive); // the wrist can't do its default command while this runs
    final double driveVelocity = 0.6;

    final Alliance allianceColor = DriverStation.getAlliance().get();
    final boolean isBlue = allianceColor == Alliance.Blue ? true : false;
    final double aimAngle = isBlue ? Math.PI / 4 : Math.PI + Math.PI / 4; // have to flip the angle

    if (isBlue) {
      drive.setPose(new Pose2d(7.2, 4.000, new Rotation2d(Math.PI)));
    } else {
      drive.setPose(new Pose2d(10.35, 4.000, new Rotation2d()));
    }

    addCommands(
        // set the pose of the robot for simulation and logging purposes
        new PrintCommand("I am beginning the autonomous."),
        new InstantCommand(
            () -> {
              if (isBlue) {
                drive.setPose(new Pose2d(7.2, 4.000, new Rotation2d(Math.PI)));
              } else {
                drive.setPose(new Pose2d(10.35, 4.000, new Rotation2d()));
              }
            }),
        // slowly drive forwards until up against the reef
        DriveCommands.joystickDrive(drive, () -> -driveVelocity, () -> 0, () -> 0)
            .withTimeout(1.15), // find this out with testing
        // eject the coral onto L1
        new PrintCommand("Ejected coral."),
        DriveCommands.joystickDrive(
                drive, () -> 0, () -> 0, () -> 0) // (testing purposes) do nothing for 2 seconds
            .withTimeout(2),
        // new SetWristRollerSpeedCommand(wrist, -1).withTimeout(0.5),
        // back out from the reef a bit
        DriveCommands.joystickDrive(drive, () -> (driveVelocity * 0.75), () -> 0, () -> 0)
            .withTimeout(0.6));
  }
}
