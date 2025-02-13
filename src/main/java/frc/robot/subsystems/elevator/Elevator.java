package frc.robot.subsystems.elevator;

import edu.wpi.first.math.controller.ProfiledPIDController;
import edu.wpi.first.math.trajectory.TrapezoidProfile;
import edu.wpi.first.wpilibj.DigitalInput;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.KDoublePreferences.PElevator;
import org.littletonrobotics.junction.AutoLogOutput;

/**
 * Elevator subsystem that controls the elevator mechanism. Uses a PID controller to drive the
 * elevator to a target height and then holds position by engaging Brake mode.
 */
public class Elevator extends SubsystemBase {
  @AutoLogOutput private double currentHeight = 0.0;
  @AutoLogOutput private double targetHeight = 0.0;
  @AutoLogOutput private boolean isOnTarget = false;

  private final ElevatorModuleIO moduleIO;
  private final DigitalInput homeSwitch; // Limit switch used for homing (assumed normally closed)
  private final ProfiledPIDController pid;
  private boolean manualOverride = false;

  /**
   * Constructor.
   *
   * @param moduleIO Implementation of the hardware interface.
   * @param homeSwitch Digital input for the home limit switch.
   */
  public Elevator(ElevatorModuleIO moduleIO, DigitalInput homeSwitch) {
    this.moduleIO = moduleIO;
    this.homeSwitch = homeSwitch;
    pid =
        new ProfiledPIDController(
            PElevator.proportional.getValue(),
            PElevator.integral.getValue(),
            PElevator.derivative.getValue(),
            new TrapezoidProfile.Constraints(
                PElevator.speedlimit.getValue(), PElevator.accelerationLimit.getValue()));
    // Start with the current height as target.
    targetHeight = moduleIO.getHeightMeters();
    pid.setGoal(targetHeight);
  }

  @Override
  public void periodic() {
    // Update sensor data.
    moduleIO.updateInputs();
    currentHeight = moduleIO.getHeightMeters();
    // Logger.processInputs("Elevator", this);

    // If the home switch is activated (returns false when pressed), reset encoders.
    if (!homeSwitch.get()) {
      moduleIO.resetEncoder();
      currentHeight = 0.0;
      targetHeight = 0.0;
      pid.reset(0.0);
    }

    // If manual override is active, do not run PID.
    if (manualOverride) {
      return;
    }

    // Clamp target height to maximum allowed.
    double maxHeight = PElevator.MaxHeight.getValue();
    if (targetHeight > maxHeight) {
      targetHeight = maxHeight;
      pid.setGoal(maxHeight);
    }

    // Compute PID output.
    double output = pid.calculate(currentHeight, targetHeight);
    isOnTarget = Math.abs(currentHeight - targetHeight) < PElevator.tolerance.getValue();

    // Prevent downward motion if already at home.
    if (output < 0 && !homeSwitch.get()) {
      output = 0;
    }

    // When on target, engage Brake mode and set voltage to 0; otherwise, drive with PID output.
    if (isOnTarget) {
      moduleIO.setBraked(true);
      moduleIO.setVoltage(0);
    } else {
      moduleIO.setBraked(false);
      moduleIO.setVoltage(output);
    }
  }

  /**
   * Sets a new target height for the elevator (using PID control).
   *
   * @param height Desired height in meters.
   */
  public void setTargetHeight(double height) {
    targetHeight = height;
    pid.setGoal(height);
    manualOverride = false;
  }

  /**
   * Allows manual control of the elevator (bypassing PID).
   *
   * @param speed A value (typically between -1 and 1) representing the motor output.
   */
  public void setManualSpeed(double speed) {
    manualOverride = true;
    if (speed < 0 && !homeSwitch.get()) {
      speed = 0;
    }
    moduleIO.setBraked(false);
    moduleIO.setVoltage(speed);
  }

  /** For more accuracy, Re-engages PID control to hold the current position. */
  public void holdPositionPID() {
    manualOverride = false;
    targetHeight = currentHeight;
    pid.setGoal(currentHeight);
  }

  /** Hold position using break. */
  public void holdPositionBreak() {
    manualOverride = true;
    targetHeight = currentHeight;
    pid.setGoal(currentHeight);
  }

  /**
   * @return The current elevator height in meters.
   */
  public double getCurrentHeight() {
    return currentHeight;
  }

  /**
   * @return True if the elevator is on target (within tolerance), false otherwise.
   */
  public boolean isOnTarget() {
    return isOnTarget;
  }
}
