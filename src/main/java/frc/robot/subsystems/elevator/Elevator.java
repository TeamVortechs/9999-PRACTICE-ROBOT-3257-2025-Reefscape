/*package frc.robot.subsystems.elevator;

import edu.wpi.first.math.controller.ProfiledPIDController;
import edu.wpi.first.math.trajectory.TrapezoidProfile;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.KDoublePreferences.PElevator;
import org.littletonrobotics.junction.AutoLogOutput;
import org.littletonrobotics.junction.Logger;

// TODO: ADD WEIGHT MAP CODE
// I PROBABLY WILL WANT TO REFACTOR ALL OF THIS CODE INTO DIFFERENT CODE FOR ELEVATOR MODULE
public class Elevator extends SubsystemBase {
  // VARIABLES

  // Variables the encompass the elevator as a whole. The annotation means it is automatically
  // logged in advantage kti
  @AutoLogOutput private double currentHeight = 0;

  @AutoLogOutput private double targetHeight = 0;

  @AutoLogOutput private boolean isOnTarget = false;

  private ElevatorModuleIO elevatorModuleIO;

  private static double Stage2 = 20.4999634796;

  private static double Stage3 = 33.2499634796;

  private static double Stage4 = 72;

  private ElevatorModuleIOInputsAutoLogged inputs = new ElevatorModuleIOInputsAutoLogged();

  private ProfiledPIDController PID =
      new ProfiledPIDController(
          PElevator.proportional.getValue(),
          PElevator.integral.getValue(),
          PElevator.derivative.getValue(),
          new TrapezoidProfile.Constraints(
              PElevator.speedlimit.getValue(), PElevator.accelerationLimit.getValue()));

  public Elevator(ElevatorModuleIO elevatorModuleIO) {
    this.elevatorModuleIO = elevatorModuleIO;
  }

  // PID controller so we don't need to do the logic ourselves. It just gets all of it's values from
  // preferences

  // LOGIC
  @Override
  public void periodic() {
    // skeleton for later:

    // logging

    elevatorModuleIO.updateInputs(inputs);
    Logger.processInputs("Elevator", inputs);

    // calculate the needed position of each elevator
    currentHeight = elevatorModuleIO.getHeightMeters();
    double diffHeight = targetHeight - currentHeight;

    // if (diffHeight < 0.1) return;

    double elevatorSpeed = PID.calculate(diffHeight);

    // individually move each elevator to that position

    if (targetHeight > PElevator.MaxHeight.getValue())
      targetHeight = PElevator.MaxHeight.getValue();

    if ((PElevator.MaxHeight.getValue() - currentHeight) < 0.1) {
      elevatorModuleIO.setSpeed(elevatorSpeed);
    } else {
      elevatorModuleIO.setSpeed(-0.05);
      // elevatorModuleIO.setBraked(true);
      System.out.println("TOO HIGH");
    }
    // finish
  }

  public Command runCurrentZeroing() {
    System.out.println("Elevator is Homed");
    return this.run(() -> elevatorModuleIO.setVoltage(-0.05))
        .until(() -> (inputs.elevatorMotor1CurrentAmps > 40))
        .finallyDo(() -> elevatorModuleIO.resetEncoder());
  }

  // HELPER
  // gets the total height of all the added modules
  public double getHeight() {
    return elevatorModuleIO.getHeightMeters();
  }

  // public void setPosition(){
  // elevatorModuleIO.setPostion(62.8318531);
  // }

  // GETTER/SETTER(simple)
  // sets the heihgt of the elevator using the pid system

  public void setTargetHeight(double height) {
    this.targetHeight = height;
    if ((PElevator.MaxHeight.getValue() - currentHeight) < 0.1) {
      System.out.println("Elevator is below Max Height");
    } else {

      targetHeight = PElevator.MaxHeight.getValue();
      System.out.println("TOO HIGH");
    }
  }

  // sets the height of the elevator but uses an enum to make it more expandable
  public void setTargetHeight(ElevatorLevel level) {
    setTargetHeight(level.getHeight());
  }

  // gets the height that the pid loop is going to
  public double getTargetHeight() {
    return targetHeight;
  }

  // returns wether or not the elevator is currently on it's target or still trying to path to it
  public boolean isOnTarget() {
    // just checks to see if the difference is low enough
    return Math.abs(currentHeight - targetHeight) < 0.04
        || Math.abs(currentHeight - targetHeight) > 0.04;
  }

  // enum for each level that the elevator could be
  public enum ElevatorLevel {
    FIRST_LEVEL(Stage2),
    SECOND_LEVEL(Stage3),
    THIRD_LEVEL(Stage4);

    private double height;

    private ElevatorLevel(double height) {
      this.height = height;
    }

    public double getHeight() {
      return height;
    }
  }
}
/* */
