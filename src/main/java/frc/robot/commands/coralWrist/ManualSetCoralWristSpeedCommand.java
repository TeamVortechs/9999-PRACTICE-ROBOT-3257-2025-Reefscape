package frc.robot.commands.coralWrist;

import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.subsystems.CoralWrist.CoralWrist;
import java.util.function.DoubleSupplier;

/*
Names
brief description
 */
public class ManualSetCoralWristSpeedCommand extends Command {
  @SuppressWarnings({"PMD.UnusedPrivateField", "PMD.SingularField"})

  /**
   * Creates a new ExampleCommand.
   *
   * @param subsystem The subsystem used by this command.
   */
  private DoubleSupplier supplierspeed;

  private CoralWrist wrist;

  private double speed;

  public ManualSetCoralWristSpeedCommand(CoralWrist wrist, DoubleSupplier supplierspeed) {
    this.wrist = wrist;
    this.supplierspeed = supplierspeed;
    addRequirements(wrist);
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {
    wrist.setManualSpeed(supplierspeed.getAsDouble());
    System.out.println("Manual Mode is in session");
  }

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {}

  // Called once the command ends or is interrupted.
  @Override
  public void end(boolean interrupted) {
    wrist.setManualSpeed(0);
  }

  // Returns true when the command should end.
  @Override
  public boolean isFinished() {
    return false;
  }
}
