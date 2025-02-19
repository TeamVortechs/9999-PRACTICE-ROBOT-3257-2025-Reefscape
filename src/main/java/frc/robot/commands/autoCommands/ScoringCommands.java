package frc.robot.commands.autoCommands;

import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.KDoublePreferences.PElevator;
import frc.robot.commands.elevator.SetElevatorPresetCommand;
import frc.robot.commands.pathfindingCommands.PathfindToClosestDepotCommand;
import frc.robot.commands.wrist.SetWristTargetAngleCommand;
import frc.robot.subsystems.drive.Drive;
import frc.robot.subsystems.elevator.Elevator;
import frc.robot.subsystems.wrist.Wrist;
import frc.robot.subsystems.wrist.Wrist.WristAngle;

public class ScoringCommands {
  public static Command prepForScoring(int level, Wrist wrist, Elevator elevator) {
    switch (level) {
      case 1:
        return new SetElevatorPresetCommand(elevator, wrist, PElevator.FirstLevel.getValue())
            .alongWith(new SetWristTargetAngleCommand(wrist, WristAngle.STAGE1_ANGLE.getAngle()));

      case 2:
        return new SetElevatorPresetCommand(elevator, wrist, PElevator.SecondLevel.getValue())
            .alongWith(new SetWristTargetAngleCommand(wrist, WristAngle.STAGE1_ANGLE.getAngle()));

      default:
        return null;
    }
  }

  // public static Command postScore(int level, Wrist wrist, Elevator elevator) {
  // return prepForScoring(level, wrist, elevator);
  // // }

  // public static Command prepForScoreAutoPath(
  //     int level, Wrist wrist, Elevator elevator, Drive drive) {
  //   return prepForScoring(level, wrist, elevator)
  //       .alongWith(new PathfindToClosestDepotCommand(drive));
  // }
}
