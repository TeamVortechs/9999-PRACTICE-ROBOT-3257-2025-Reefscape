package frc.robot.commands;

import edu.wpi.first.wpilibj.GenericHID.RumbleType;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.button.CommandXboxController;

/*
Names
brief description
 */
public class ControllerVibrateCommand extends Command {
  @SuppressWarnings({"PMD.UnusedPrivateField", "PMD.SingularField"})

  /**
   * Creates a new ExampleCommand.
   *
   * @param subsystem The subsystem used by this command.
   */
  private CommandXboxController controller;

  private double totalTime;
  private double intensity;

  private double curTime = 0;

  public ControllerVibrateCommand(CommandXboxController controller, double time, double intensity) {
    // addRequirements(null);
    this.controller = controller;

    this.totalTime = time;
    this.intensity = intensity;
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {
    controller.setRumble(RumbleType.kBothRumble, intensity);
    curTime = 0;
  }

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {
    curTime++;

    if (curTime > totalTime) {
      controller.setRumble(RumbleType.kBothRumble, 0);
    }
  }

  // Called once the command ends or is interrupted.
  @Override
  public void end(boolean interrupted) {}

  // Returns true when the command should end.
  @Override
  public boolean isFinished() {
    return (curTime > totalTime);
  }
}
