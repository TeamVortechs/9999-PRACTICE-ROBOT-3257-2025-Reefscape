package frc.robot.commands.pathfindingCommands;

import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.button.CommandXboxController;
import frc.robot.subsystems.drive.Drive;

/*
Names
brief description
 */
// THIS CLASS WILL BREAK THE ROBOT
public class AlignWithClosestIntakeCommand extends Command {
  @SuppressWarnings({"PMD.UnusedPrivateField", "PMD.SingularField"})

  /**
   * Creates a new ExampleCommand.
   *
   * @param subsystem The subsystem used by this command.
   */
  private Drive drive;

  private int targetPoseID = 0;

  private boolean lockedIn = false;

  private Command[] intakeAligningCommands;

  private CommandXboxController controller;

  private double speedModifier;

  public AlignWithClosestIntakeCommand(Drive drive, CommandXboxController controller, double speedModifier) {
    // addRequirements(null);
    this.drive = drive;
    this.controller = controller;
    intakeAligningCommands = new Command[2];
    this.speedModifier = speedModifier;
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {
    for (int i = 0; i < intakeAligningCommands.length; i++) {
      intakeAligningCommands[i] = PathfindingCommands.alignToIntakeCommand(i, drive, controller, speedModifier);
    }
  }

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {
    int curPoseID = PathfindingCommands.getClosestIntakePath(drive.getPose());

    if (!lockedIn) {
      targetPoseID = curPoseID;
      
      intakeAligningCommands[targetPoseID].schedule();
    }

    if (targetPoseID != curPoseID) {
      targetPoseID = curPoseID;
      intakeAligningCommands[targetPoseID].schedule();
    }
  }

  // Called once the command ends or is interrupted.
  @Override
  public void end(boolean interrupted) {
    intakeAligningCommands[targetPoseID].cancel();

    lockedIn = false;
  }

  // Returns true when the command should end.
  @Override
  public boolean isFinished() {
    return intakeAligningCommands[targetPoseID].isFinished();
  }
}
