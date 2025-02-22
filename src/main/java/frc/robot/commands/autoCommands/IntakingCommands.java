package frc.robot.commands.autoCommands;

import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.Constants;
import frc.robot.KDoublePreferences.PElevator;
import frc.robot.commands.elevator.SetElevatorPresetCommand;
import frc.robot.commands.wrist.IntakeWristCommand;
import frc.robot.commands.wrist.SetWristTargetAngleCommand;
import frc.robot.subsystems.elevator.Elevator;
import frc.robot.subsystems.wrist.Wrist;
import frc.robot.subsystems.wrist.Wrist.WristAngle;

public class IntakingCommands {
  // moves wrist and elevator into position then rolls the wrist until object is detected
  public static Command intakeCommand(Wrist wrist, Elevator elevator) {
    return new SetElevatorPresetCommand(elevator, wrist, Constants.Elevator.MIN_HEIGHT)
        .andThen(new SetWristTargetAngleCommand(wrist, WristAngle.INTAKE_ANGLE.getAngle()))
        .alongWith(new IntakeWristCommand(wrist, -0.2));
  }

  public static Command prepForIntakeCommand(Wrist wrist, Elevator elevator) {
    return new SetElevatorPresetCommand(elevator, wrist, Constants.Elevator.MIN_HEIGHT)
        .andThen(new SetWristTargetAngleCommand(wrist, WristAngle.INTAKE_ANGLE.getAngle()));
  }

  // does the intake command but automatically aligns at the same time
  // public static Command autoAligningIntakedCommand(
  //     Wrist wrist, Elevator elevator, Drive drive, CommandXboxController controller) {
  //   return intakeCommand(wrist, elevator)
  //       .alongWith(new AlignWithClosestIntakeCommand(drive, controller, 1));
  // }

  // // does the intake command and moves the robot to position at the same time
  // public static Command autoMovingIntakeCommand(Wrist wrist, Elevator elevator, Drive drive) {
  //   return intakeCommand(wrist, elevator).alongWith(new PathfindToClosestIntakeCommand(drive));
  // }
}
