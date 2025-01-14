package frc.robot.subsystems.elevator;

import edu.wpi.first.math.controller.ProfiledPIDController;
import edu.wpi.first.math.trajectory.TrapezoidProfile;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.KDoublePreferences;
import frc.robot.KDoublePreferences.PElevator;
import org.littletonrobotics.junction.AutoLogOutput;
import org.littletonrobotics.junction.Logger;

// TODO: ADD WEIGHT MAP CODE
// I PROBABLY WILL WANT TO REFACTOR ALL OF THIS CODE INTO DIFFERENT CODE FOR ELEVATOR MODULE
public class Elevator extends SubsystemBase {
  // VARIABLES

  // Variables the encompass the elevator as a whole. The annotation means it is automatically

  @AutoLogOutput private double targetHeight = 0;

  @AutoLogOutput private boolean isOnTarget = false;

  @AutoLogOutput private double height = 0;

  private ElevatorModuleIO elevatorModuleIO;

  private ElevatorModuleIOInputsAutoLogged inputs = new ElevatorModuleIOInputsAutoLogged();

  private ProfiledPIDController PID =
      new ProfiledPIDController(
          PElevator.proportional.getValue(),
          PElevator.integral.getValue(),
          PElevator.derivative.getValue(),
          new TrapezoidProfile.Constraints(targetHeight, getHeight()));

  public Elevator(ElevatorModuleIO elevatorModuleIO) {
    this.elevatorModuleIO = elevatorModuleIO;
  }

  // PID controller so we don't need to do the logic ourselves. It just gets all of it's values from
  // preferences
  ProfiledPIDController elevatorPID =
      new ProfiledPIDController(
          KDoublePreferences.PElevator.proportional.getValue(),
          KDoublePreferences.PElevator.integral.getValue(),
          KDoublePreferences.PElevator.derivative.getValue(),
          new TrapezoidProfile.Constraints(
              KDoublePreferences.PElevator.speedlimit.getValue(),
              KDoublePreferences.PElevator.accelerationLimit.getValue()));

  // LOGIC
  @Override
  public void periodic() {
    // skeleton for later:

    height = elevatorModuleIO.getHeightMeters();

    // logging
    for(int i = 0; i < elevatorModulesIO.length; i++) {
        elevatorModulesIO[i].updateInputs(inputs);
        Logger.processInputs("Elevator", inputs);
    }

    // calculate the needed position of each elevator

    double diffHeight = targetHeight - getHeight();

    if (diffHeight < 0.1) return;

    double elevatorSpeed = PID.calculate(diffHeight);

    // individually move each elevator to that position

    elevatorModuleIO.setSpeed(elevatorSpeed);
    // finish
  }

  // HELPER
  // gets the total height of all the added modules
  public double getHeight() {

    double height = 0.0;

    for(int i = 0; i < elevatorModulesIO.length; i++) {
        height += elevatorModulesIO[i].getHeightMeters();
    }
    return height;
  }

  // GETTER/SETTER(simple)
  // sets the heihgt of the elevator using the pid system
  public void setTargetHeight(double height) {
    this.targetHeight = height;
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
    return Math.abs(getHeight() - targetHeight) < 0.04;
  }

  // enum for each level that the elevator could be
  public enum ElevatorLevel {
    FIRST_LEVEL(1.0),
    SECOND_LEVEL(2.0),
    THIRD_LEVEL(3.0);

    private double height;

    private ElevatorLevel(double height) {
      this.height = height;
    }

    public double getHeight() {
      return height;
    }
  }
}
