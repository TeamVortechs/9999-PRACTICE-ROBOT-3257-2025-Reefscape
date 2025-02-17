package frc.robot.commands.pathfindingCommands;

import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.subsystems.drive.Drive;

/*
Names
brief description
 */
// THIS CLASS WILL BREAK THE ROBOT
public class PathfindToClosestIntakeCommand extends Command {
  @SuppressWarnings({"PMD.UnusedPrivateField", "PMD.SingularField"})

  /**
   * Creates a new ExampleCommand.
   *
   * @param subsystem The subsystem used by this command.
   */
  private Drive drive;

  private int targetPoseID = 0;

  private boolean lockedIn = false;

  private Command[] intakePathCommands;

  public PathfindToClosestIntakeCommand(Drive drive) {
    // addRequirements(null);
    this.drive = drive;
    intakePathCommands = new Command[2];
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {
    for (int i = 0; i < intakePathCommands.length; i++) {
      intakePathCommands[i] = PathfindingCommands.pathfindToIntakeCommand(i);
    }
  }

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {
    int curPoseID = PathfindingCommands.getClosestIntakePath(drive.getPose());

    if (!lockedIn) {
      targetPoseID = curPoseID;
      intakePathCommands[targetPoseID].schedule();
    }

    if (targetPoseID != curPoseID) {
      targetPoseID = curPoseID;
      intakePathCommands[targetPoseID].schedule();
      ;
    }
  }

  // Called once the command ends or is interrupted.
  @Override
  public void end(boolean interrupted) {
    intakePathCommands[targetPoseID].cancel();

    lockedIn = false;
  }

  // Returns true when the command should end.
  @Override
  public boolean isFinished() {
    return intakePathCommands[targetPoseID].isFinished();
  }
}
