package frc.robot.commands.pathfindingCommands;

import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.subsystems.drive.Drive;

/**
 * PathfindToClosestDepotCommand is a dynamic command that continuously monitors the robot's pose
 * and selects an appropriate pathfinding command to navigate to the closest depot.
 *
 * <p>Improvements Made: - Removed pre-created command array and replaced it with dynamic command
 * creation. - Removed the 'lockedIn' flag; instead, the target is managed via targetPoseID. -
 * Properly cancels the active command when a new target is selected or when the command ends.
 */
public class PathfindToClosestDepotCommand extends Command {

  private final Drive drive;
  private final boolean left;
  private Command activeCommand; // Dynamically created command instance
  // private int targetPoseID = -1; // -1 indicates no target selected yet

  private boolean scheduledPath = false;

  /**
   * Constructs a new PathfindToClosestDepotCommand.
   *
   * @param drive the drive subsystem.
   * @param left whether to use left-side depot paths.
   */
  public PathfindToClosestDepotCommand(Drive drive, boolean left) {
    this.drive = drive;
    this.left = left;
    // Declare dependency on the drive subsystem.
    addRequirements(drive);
  }

  @Override
  public void initialize() {
    // Reset state on initialization.
    // targetPoseID = -1;
    activeCommand = null;
    scheduledPath = false;
  }

  @Override
  public void execute() {
    // Determine the closest depot path index based on the current robot pose.
    // int curPoseID = PathfindingCommands.getClosestDepotPath(drive.getPose(), left);

    // If the depot target has changed, cancel the previous command and schedule a new one.
    // if (curPoseID != targetPoseID) {
    //   if (activeCommand != null && activeCommand.isScheduled()) {
    //     activeCommand.cancel();
    //   }
    //   targetPoseID = curPoseID;
    //   // Create a new command instance using the factory method.
    //   activeCommand = PathfindingCommands.pathfindToDepotCommand(targetPoseID, left);
    //   activeCommand.schedule();
    // }

    if (scheduledPath == false) {
      int curPoseID = PathfindingCommands.getClosestDepotPath(drive.getPose(), left);

      if (activeCommand != null && activeCommand.isScheduled()) {
        activeCommand.cancel();
      }

      activeCommand = PathfindingCommands.pathfindToDepotCommand(curPoseID, left);
      activeCommand.schedule();

      scheduledPath = true;
    }
  }

  @Override
  public void end(boolean interrupted) {
    // Cancel the active command if it is still scheduled.
    // if (activeCommand != null && activeCommand.isScheduled()) {
    activeCommand.cancel();
    // }
  }

  @Override
  public boolean isFinished() {
    // Consider this command finished when the active command has completed.
    return activeCommand != null && activeCommand.isFinished();
  }
}
