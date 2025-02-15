package frc.robot.commands;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import frc.robot.KDoublePreferences.PElevator;
import frc.robot.commands.wrist.SetWristTargetAngleCommand;
import frc.robot.subsystems.drive.Drive;
import frc.robot.subsystems.wrist.Wrist;

public class IntakingCommands {
  // moves wrist and elevator into position then rolls the wrist until object is detected
  // public static Command intakeCommand(Wrist wrist, Elevator elevator) {
  //   return new SetElevatorCommand(elevator, PElevator.intakeLevel.getValue())
  //       .alongWith(new SetWristTargetAngleCommand(wrist, wrist.intakeAngle))
  //       .andThen(wrist.rollWristUntilDetectedCommand(0.4, 0.1));
  // }

  // // does the intake command but automatically aligns at the same time
  // public static Command autoAligningIntakedCommand(Wrist wrist, Elevator elevator, Drive drive) {
  //   return intakeCommand(wrist, elevator)
  //       .alongWith(
  //           Commands.runOnce(
  //                   () ->
  //                       drive.setPose(
  //                           new Pose2d(drive.getPose().getTranslation(), new Rotation2d())),
  //                   drive)
  //               .ignoringDisable(true));
  // }

  // // does the intake command and moves the robot to position at the same time
  // public static Command autoMovingIntakeCommand(Wrist wrist, Elevator elevator, Drive drive) {
  //   return intakeCommand(wrist, elevator);
  //   // this needs work
  //   // .alongWith(PathfindingCommands.path)
  // }
}
