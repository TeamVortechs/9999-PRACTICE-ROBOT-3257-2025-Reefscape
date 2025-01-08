package frc.robot.commands;

import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.subsystems.elevator.Elevator;
import frc.robot.subsystems.elevator.Elevator.ElevatorLevel;

/*
Ben
Sets the location of the elevator to the given setpoint
 */
public class SetElevatorCommand extends Command {
  @SuppressWarnings({"PMD.UnusedPrivateField", "PMD.SingularField"})

  /**
   * Creates a new ExampleCommand.
   *
   * @param subsystem The subsystem used by this command.
   */

   private double wantedHeight;
   private Elevator elevator;


  public SetElevatorCommand(double wantedHeight, Elevator elevator) {
    this.wantedHeight = wantedHeight;
    this.elevator = elevator;
  }

  public SetElevatorCommand(ElevatorLevel elevatorLevel, Elevator elevator) {
    this.wantedHeight = elevatorLevel.getHeight();
    this.elevator = elevator;
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {
    elevator.setTargetHeight(wantedHeight);
  }

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {

  }

  // Called once the command ends or is interrupted.
  @Override
  public void end(boolean interrupted) {

  }

  // Returns true when the command should end.
  //finish either if the elevator has reached it's height or if the target height changed
  @Override
  public boolean isFinished() {

    //just a way to ask if the doubles are not equal which works for innacuracies
    if(Math.abs(wantedHeight - elevator.getTargetHeight()) > 0.0001) {
        return true;
    }

    //returns true if the heihgt of the elevator is really close to the wanted height
    if(Math.abs(wantedHeight - elevator.getHeight()) > 0.1) {
        return true;
    }

    return false;
  }
}