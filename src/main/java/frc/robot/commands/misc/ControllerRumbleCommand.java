package frc.robot.commands.misc;

import edu.wpi.first.wpilibj.GenericHID.RumbleType;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.button.CommandXboxController;

/*
Names
brief description
 */
public class ControllerRumbleCommand extends Command {
  @SuppressWarnings({"PMD.UnusedPrivateField", "PMD.SingularField"})

  /**
   * Creates a new ExampleCommand.
   *
   * @param subsystem The subsystem used by this command.
   */

   private int ticks;
   private int totalTicks;

   private double intensity;

   private CommandXboxController controller;
  public ControllerRumbleCommand(int ticks, double intensity, CommandXboxController controller) {
    // addRequirements(null);

    this.intensity = intensity;
    this.totalTicks = ticks;
    this.controller = controller;
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {
    ticks = 0;

    controller.setRumble(RumbleType.kBothRumble, intensity);
  }

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {
    ticks++;
  }

  // Called once the command ends or is interrupted.
  @Override
  public void end(boolean interrupted) {
    controller.setRumble(RumbleType.kBothRumble, 0);
  }

  // Returns true when the command should end.
  @Override
  public boolean isFinished() {
    return (ticks > totalTicks);
  }
}
