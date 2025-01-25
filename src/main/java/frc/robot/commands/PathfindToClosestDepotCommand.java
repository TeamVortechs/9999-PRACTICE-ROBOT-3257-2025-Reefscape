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
  public void initialize() {
    depotPathCommands = PathfindingCommands.getPathfindingCommands();
  }

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {
    int testPoseID = PathfindingCommands.getClosestDepotPath(drive.getPose());
    System.out.println("testposeID: " + testPoseID);

    if (!lockedIn) {
      depotPathCommands[targetPoseID].schedule();
      lockedIn = true;
    }

    if (testPoseID != targetPoseID) {

      depotPathCommands[targetPoseID].cancel();

      targetPoseID = testPoseID;
      depotPathCommands[targetPoseID].schedule();
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
