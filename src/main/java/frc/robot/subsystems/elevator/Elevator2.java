package frc.robot.subsystems.elevator;

import edu.wpi.first.math.controller.ProfiledPIDController;
import edu.wpi.first.math.trajectory.TrapezoidProfile;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.KDoublePreferences.PElevator;
import org.littletonrobotics.junction.AutoLogOutput;
import org.littletonrobotics.junction.Logger;

public class Elevator2 extends SubsystemBase {

  private ElevatorModuleIO elevatorModuleIO;

  private ElevatorModuleIOInputsAutoLogged inputs = new ElevatorModuleIOInputsAutoLogged();

  @AutoLogOutput private double currentHeight = 0;

  @AutoLogOutput private double targetHeight = 0;

  @AutoLogOutput private boolean isOnTarget = false;

  private static double Stage2 = 20.4999634796;

  private static double Stage3 = 33.2499634796;

  private static double Stage4 = 72;

  private ProfiledPIDController PID =
      new ProfiledPIDController(
          PElevator.proportional.getValue(),
          PElevator.integral.getValue(),
          PElevator.derivative.getValue(),
          new TrapezoidProfile.Constraints(
              PElevator.speedlimit.getValue(), PElevator.accelerationLimit.getValue()));

  public Elevator2(ElevatorModuleIO elevatorModuleIO) {
    this.elevatorModuleIO = elevatorModuleIO;
  }

  public Command runCurrentZeroing() {
    System.out.println("Elevator is Homed");
    return this.run(() -> elevatorModuleIO.setVoltage(-0.05))
        .until(() -> (inputs.elevatorMotor1CurrentAmps > 40))
        .finallyDo(() -> elevatorModuleIO.resetEncoder());
  }

  @Override
  public void periodic() {
    elevatorModuleIO.updateInputs(inputs);
    Logger.processInputs("Elevator2", inputs);
    // calculate the needed position of each elevator
    currentHeight = elevatorModuleIO.getHeightMeters();
    double diffHeight = targetHeight - currentHeight;

    // if (diffHeight < 0.1) return;

    double elevatorSpeed = PID.calculate(diffHeight);

    // individually move each elevator to that position
    // makes sure that target height isn't set towards max height
    if (targetHeight > PElevator.MaxHeight.getValue())
      targetHeight = PElevator.MaxHeight.getValue();
    // blocks the motors from reaching max height
    if ((PElevator.MaxHeight.getValue() - currentHeight) < 0.1) {
      elevatorModuleIO.setSpeed(elevatorSpeed);
    } else if ((currentHeight - PElevator.MinHeight.getValue()) > 0.05) {
      System.out.println("TOO HIGH");
      elevatorModuleIO.setSpeed(0.1);
    } else {
      elevatorModuleIO.setSpeed(-0.05);
      System.out.println("TOO HIGH");
    }
    // finish
  }

  public void setTargetHeight(double height) {
    // sets target height
    this.targetHeight = height;
    // checks if elevator reaches max or not
    if ((PElevator.MaxHeight.getValue() - currentHeight) < 0.1) {
      System.out.println("Elevator is below Max Height");
    } else if ((currentHeight - PElevator.MinHeight.getValue()) > 0.05) {
      System.out.println("Elevator is too LOW");
    } else {
      targetHeight = PElevator.MaxHeight.getValue();
      System.out.println("TOO HIGH");
    }
  }

  public double getHeight() {
    SmartDashboard.putNumber("Elevator Height", currentHeight);
    SmartDashboard.putNumber("Target Height", targetHeight);
    SmartDashboard.putBoolean("At Target", isOnTarget);
    return elevatorModuleIO.getHeightMeters();
  }

  public void setSpeed(double speed) {
    elevatorModuleIO.setSpeed(speed);
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
