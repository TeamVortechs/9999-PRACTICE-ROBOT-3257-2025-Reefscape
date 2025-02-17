package frc.robot.commands.autoCommands;

import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import edu.wpi.first.wpilibj2.command.button.CommandXboxController;
import frc.robot.KDoublePreferences.PElevator;
import frc.robot.commands.elevator.SetElevatorPresetCommand;
import frc.robot.commands.pathfindingCommands.AlignWithClosestIntakeCommand;
import frc.robot.commands.pathfindingCommands.PathfindToClosestIntakeCommand;
import frc.robot.commands.wrist.IntakeWristCommand;
import frc.robot.commands.wrist.SetWristTargetAngleCommand;
import frc.robot.subsystems.drive.Drive;
import frc.robot.subsystems.elevator.Elevator;
import frc.robot.subsystems.wrist.Wrist;
import frc.robot.subsystems.wrist.Wrist.WristAngle;

public class IntakingCommands {
  // moves wrist and elevator into position then rolls the wrist until object is detected
  public static Command intakeCommand(Wrist wrist, Elevator elevator) {
    return new SetElevatorPresetCommand(elevator, wrist, PElevator.MinHeight.getValue())
        .alongWith(new SetWristTargetAngleCommand(wrist, WristAngle.INTAKE_ANGLE.getAngle()))
        .andThen(new IntakeWristCommand(wrist, -0.2));
  }

  // does the intake command but automatically aligns at the same time
  public static Command autoAligningIntakedCommand(Wrist wrist, Elevator elevator, Drive drive, CommandXboxController controller) {
    return intakeCommand(wrist, elevator)
    .alongWith(new AlignWithClosestIntakeCommand(drive, controller, 1));
        
  }

  // does the intake command and moves the robot to position at the same time
  public static Command autoMovingIntakeCommand(Wrist wrist, Elevator elevator, Drive drive) {
    return intakeCommand(wrist, elevator)
    .alongWith(new PathfindToClosestIntakeCommand(drive));

  }
}
