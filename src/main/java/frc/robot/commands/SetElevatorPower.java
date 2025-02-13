package frc.robot.commands;

import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.subsystems.elevator.Elevator2;

/*
Names
brief description
 */
public class SetElevatorPower extends Command {
  @SuppressWarnings({"PMD.UnusedPrivateField", "PMD.SingularField"})

  /**
   * Creates a new ExampleCommand.
   *
   * @param subsystem The subsystem used by this command.
   */
  private Elevator2 elevator2;

  private double Speed;

  public SetElevatorPower(Elevator2 elevator2, double Speed) {
    this.elevator2 = elevator2;
    this.Speed = Speed;
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {
    elevator2.setSpeed(Speed);
  }

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {}

  // Called once the command ends or is interrupted.
  @Override
  public void end(boolean interrupted) {
    elevator2.setSpeed(0.0);
  }

  // Returns true when the command should end.
  @Override
  public boolean isFinished() {
    return false;
  }
}
