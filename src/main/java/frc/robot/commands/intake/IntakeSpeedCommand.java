package frc.robot.commands.intake;

import edu.wpi.first.wpilibj.DigitalInput;
import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.subsystems.Intake.Intake;

/*
Names
brief description
 */
public class IntakeSpeedCommand extends Command {
  @SuppressWarnings({"PMD.UnusedPrivateField", "PMD.SingularField"})
  private final Intake intake;

  private double speed;
  private DigitalInput limitSwitch;

  /**
   * Creates a new ExampleCommand.
   *
   * @param subsystem The subsystem used by this command.
   */
  public IntakeSpeedCommand(Intake intake, double speed, DigitalInput limitSwitch) {
    // addRequirements(null);
    this.intake = intake;
    this.speed = speed;
    this.limitSwitch = limitSwitch;
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {
    intake.SetMotorSpeed(speed);
  }

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {
    if (intake.isDetected(limitSwitch)) {
      intake.SetMotorSpeed(0);
    }
  }

  // Called once the command ends or is interrupted.
  @Override
  public void end(boolean interrupted) {
    intake.SetMotorSpeed(0);
  }

  // Returns true when the command should end.
  @Override
  public boolean isFinished() {
    return false;
  }
}
