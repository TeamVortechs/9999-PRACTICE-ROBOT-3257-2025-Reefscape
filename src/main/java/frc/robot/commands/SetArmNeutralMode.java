package frc.robot.commands;

import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.subsystems.arm.Arm;

/*
Names
brief description
 */
public class SetArmNeutralMode extends Command {
  @SuppressWarnings({"PMD.UnusedPrivateField", "PMD.SingularField"})

  /**
   * Creates a new ExampleCommand.
   *
   * @param subsystem The subsystem used by this command.
   */
  private boolean mode;
  private Arm wrist;

  //sets the idle mode of the motor
  public SetArmNeutralMode(boolean mode, Arm wrist) {
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
