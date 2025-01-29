package frc.robot.commands;

import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.subsystems.drive.Drive;

/*
Names
brief description
 */
// THIS CLASS WILL BREAK THE ROBOT
public class PathfindToClosestDepotCommand extends Command {
  @SuppressWarnings({"PMD.UnusedPrivateField", "PMD.SingularField"})

  /**
   * Creates a new ExampleCommand.
   *
   * @param subsystem The subsystem used by this command.
   */
  private Drive drive;

  private int targetPoseID = 0;

  private boolean lockedIn = false;

  private Command[] depotPathCommands;

  public PathfindToClosestDepotCommand(Drive drive) {
    // addRequirements(null);
    this.drive = drive;
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {}

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {
    Pose2d testPose =
        PathfindingCommands.getDepotPose(PathfindingCommands.getClosestDepotPath(drive.getPose()));
    System.out.println(testPose.toString());
    System.out.println("target: " + targetPose2d.toString());
    if (!testPose.equals(targetPose2d)) {
      if (PathfindingCommands.getClosestDepotPath(testPose) < 0
          || PathfindingCommands.getClosestDepotPath(testPose) > 5) {
        System.out.println(
            "ERROR: index out of bounds (Pathfiding to closest depot depot command, execute function)");
      }

      System.out.println("redid command");

      command =
          PathfindingCommands.pathfindToDepotCommand(
              PathfindingCommands.getClosestDepotPath(drive.getPose()));

      targetPose2d =
          PathfindingCommands.getDepotPose(
              PathfindingCommands.getClosestDepotPath(drive.getPose()));
    }
  }

  // Called once the command ends or is interrupted.
  @Override
  public void end(boolean interrupted) {
    if (lockedIn) {
      depotPathCommands[targetPoseID].end(interrupted);
    }
  }

  // Returns true when the command should end.
  @Override
  public boolean isFinished() {
    return false;
  }
}
