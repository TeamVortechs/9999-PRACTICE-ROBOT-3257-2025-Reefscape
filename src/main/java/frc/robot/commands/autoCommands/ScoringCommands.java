package frc.robot.commands.autoCommands;

import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.Constants;
import frc.robot.KDoublePreferences.PElevator;
import frc.robot.commands.elevator.SetElevatorPresetCommand;
import frc.robot.commands.wrist.SetWristTargetAngleCommand;
import frc.robot.subsystems.elevator.Elevator;
import frc.robot.subsystems.wrist.Wrist;
import frc.robot.subsystems.wrist.Wrist.WristAngle;

public class ScoringCommands {
  public static Command prepForScoring(int level, Wrist wrist, Elevator elevator) {
    switch (level) { // bit of a misnomer here
      case 1:
        return new SetElevatorPresetCommand(elevator, wrist, Constants.Elevator.STAGE_2_LEVEL)
            .alongWith(new SetWristTargetAngleCommand(wrist, WristAngle.STAGE2_ANGLE.getAngle()));

      case 2:
        return new SetElevatorPresetCommand(elevator, wrist, Constants.Elevator.STAGE_3_LEVEL)
            .alongWith(new SetWristTargetAngleCommand(wrist, WristAngle.STAGE2_ANGLE.getAngle()));

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
