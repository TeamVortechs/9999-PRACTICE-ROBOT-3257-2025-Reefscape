package frc.robot.commands.coralWrist;

import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.subsystems.CoralWrist.CoralWrist;

/*
Names
brief description
 */
public class SetCoralWristNeutralMode extends Command {
  @SuppressWarnings({"PMD.UnusedPrivateField", "PMD.SingularField"})

  /**
   * Creates a new ExampleCommand.
   *
   * @param subsystem The subsystem used by this command.
   */
  private boolean mode;

  private CoralWrist wrist;

  public SetCoralWristNeutralMode(boolean mode, CoralWrist wrist) {
    this.mode = mode;
    this.wrist = wrist;
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {
    wrist.setBraked(mode);
  }

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {}

  // Called once the command ends or is interrupted.
  @Override
  public void end(boolean interrupted) {}

  // Returns true when the command should end.
  @Override
  public boolean isFinished() {
    return true;
  }
}
