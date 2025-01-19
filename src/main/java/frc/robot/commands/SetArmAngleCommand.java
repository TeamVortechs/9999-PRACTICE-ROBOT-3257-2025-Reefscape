package frc.robot.commands;

import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.subsystems.arm.Arm;

/*
Names
brief description
 */
public class SetArmAngleCommand extends Command {
  @SuppressWarnings({"PMD.UnusedPrivateField", "PMD.SingularField"})

  /**
   * Creates a new ExampleCommand.
   *
   * @param subsystem The subsystem used by this command.
   */
  private Arm arm;

  private double angle;

  public SetArmAngleCommand(Arm arm, double angle) {
    addRequirements(arm);
    this.arm = arm;
    this.angle = angle;
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {
    arm.setTargetAngleRad(angle);
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
    // if the target angle changed the command is finished(fnacy statement bc it's a double)
    if (Math.abs(angle - arm.getTargetAngleRad()) > 0.01) {
      return true;
    }

    return false;
  }
}
