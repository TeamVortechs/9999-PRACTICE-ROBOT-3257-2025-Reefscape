package frc.robot.commands.coralWrist;

import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.subsystems.CoralWrist.CoralWrist;

/*
Names
brief description
 */
public class SetCoralWristRollerSpeedCommand extends Command {
  @SuppressWarnings({"PMD.UnusedPrivateField", "PMD.SingularField"})

  /**
   * Creates a new ExampleCommand.
   *
   * @param subsystem The subsystem used by this command.
   */
  private double speed;

  private CoralWrist wrist;

  public SetCoralWristRollerSpeedCommand(CoralWrist wrist, double speed) {
    // addRequirements(null);
    this.wrist = wrist;
    this.speed = speed;
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {}

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {
    wrist.setRollerSpeed(speed);
  }

  // Called once the command ends or is interrupted.
  @Override
  public void end(boolean interrupted) {
    wrist.setRollerSpeed(0);
  }

  // Returns true when the command should end.
  @Override
  public boolean isFinished() {
    return false;
  }
}
