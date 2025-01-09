package frc.robot.subsystems.elevator;

import edu.wpi.first.math.controller.ProfiledPIDController;
import edu.wpi.first.math.trajectory.TrapezoidProfile;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.KDoublePreferences;
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

  private ElevatorModuleIO[] elevatorModulesIO;

  private ElevatorModuleIOInputsAutoLogged inputs = new ElevatorModuleIOInputsAutoLogged();

  public Elevator(ElevatorModuleIO[] elevatorModuleIO) {
    this.elevatorModulesIO = elevatorModuleIO;
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

    // logging
    for(int i = 0; i < elevatorModulesIO.length; i++) {
        elevatorModulesIO[i].updateInputs(inputs);
        Logger.processInputs("Elevator", inputs);
    }

    // calculate the needed position of each elevator

    // individually move each elevator to that position

    // finish
  }

  // HELPER
  // gets the total height of all the added modules
  public double getHeight() {

    double height = 0.0;

    for(int i = 0; i < elevatorModulesIO.length; i++) {
        height += elevatorModulesIO[i].getHeightMeters();
    }
    return d;
  }

  // GETTER/SETTER(simple)
  // sets the heihgt of the elevator using the pid system
  public void setTargetHeight(double height) {}

  // sets the height of the elevator but uses an enum to make it more expandable
  public void setTargetHeight(ElevatorLevel level) {
    setTargetHeight(level.getHeight());
  }

  // gets the height that the pid loop is going to
  public double getTargetHeight() {
    return 0;
  }

  // returns wether or not the elevator is currently on it's target or still trying to path to it
  public boolean isOnTarget() {
    // just checks to see if the difference is low enough
    return Math.abs(currentHeight - targetHeight) < 0.04;
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
