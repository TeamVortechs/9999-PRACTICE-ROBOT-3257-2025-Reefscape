package frc.robot.commands.coralWrist;

import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.subsystems.CoralWrist.CoralWrist;
import java.util.function.DoubleSupplier;

/*
Names
brief description
 */
public class SetCoralWristTargetAngleCommand extends Command {
  @SuppressWarnings({"PMD.UnusedPrivateField", "PMD.SingularField"})

  /**
   * Creates a new ExampleCommand.
   *
   * @param subsystem The subsystem used by this command.
   */
  private final CoralWrist wrist;

  private final DoubleSupplier targetAngle;

  public SetCoralWristTargetAngleCommand(CoralWrist wrist, DoubleSupplier targetAngle) {
    addRequirements(wrist);

    this.wrist = wrist;
    this.targetAngle = targetAngle;
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {}

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {
    // wrist.setRollerSpeed(0.2);
    wrist.setTargetAngle(targetAngle.getAsDouble());
  }

  // Called once the command ends or is interrupted.
  @Override
  public void end(boolean interrupted) {}

  // Returns true when the command should end.
  @Override
  public boolean isFinished() {
    // return wrist.isOnTarget();
    return true;
  }
}
