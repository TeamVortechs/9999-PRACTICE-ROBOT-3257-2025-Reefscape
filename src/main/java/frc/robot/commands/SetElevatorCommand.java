// package frc.robot.commands;

// import edu.wpi.first.wpilibj2.command.Command;
// import frc.robot.subsystems.elevator.Elevator2;
// import frc.robot.subsystems.elevator.Elevator2.ElevatorLevel;

// /*
// Ben
// Sets the location of the elevator to the given setpoint
//  */
// public class SetElevatorCommand extends Command {
//   @SuppressWarnings({"PMD.UnusedPrivateField", "PMD.SingularField"})

//   /**
//    * Creates a new ExampleCommand.
//    *
//    * @param subsystem The subsystem used by this command.
//    */
//   private double wantedHeight;

//   private Elevator2 elevator2;

//   public SetElevatorCommand(double wantedHeight, Elevator2 elevator2) {
//     this.wantedHeight = wantedHeight;
//     this.elevator2 = elevator2;
//   }

//   public SetElevatorCommand(ElevatorLevel elevatorLevel, Elevator2 elevator2) {
//     this.wantedHeight = elevatorLevel.getHeight();
//     this.elevator2 = elevator2;
//   }

//   // Called when the command is initially scheduled.
//   @Override
//   public void initialize() {
//     elevator2.setTargetHeight(wantedHeight);
//   }

//   // Called every time the scheduler runs while the command is scheduled.
//   @Override
//   public void execute() {}

//   // Called once the command ends or is interrupted.
//   @Override
//   public void end(boolean interrupted) {}

//   // Returns true when the command should end.
//   // finish either if the elevator has reached it's height or if the target height changed
//   @Override
//   public boolean isFinished() {

//     // just a way to ask if the doubles are not equal which works for innacuracies
//     if (Math.abs(wantedHeight - elevator2.getTargetHeight()) > 0.0001) {
//       return true;
//     }

//     // returns true if the heihgt of the elevator is really close to the wanted height
//     if (Math.abs(wantedHeight - elevator2.getHeight()) > 0.1) {
//       return true;
//     }

//     return false;
//   }
// }
