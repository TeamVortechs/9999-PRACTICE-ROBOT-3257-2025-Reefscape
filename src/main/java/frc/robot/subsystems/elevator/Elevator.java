package frc.robot.subsystems.elevator;

import edu.wpi.first.math.controller.ElevatorFeedforward;
import edu.wpi.first.math.controller.ProfiledPIDController;
import edu.wpi.first.math.trajectory.TrapezoidProfile;
import edu.wpi.first.wpilibj.DigitalInput;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.KDoublePreferences.PElevator;
import org.littletonrobotics.junction.AutoLogOutput;

/**
 * Elevator subsystem that controls the robot's lifting mechanism.
 * Utilizes PID control and feedforward for smooth and accurate movement.
 * Prevents unsafe movement using limit switches and software constraints.
 */
public class Elevator extends SubsystemBase {
  
  @AutoLogOutput private double currentHeight = 0.0;
  @AutoLogOutput private double targetHeight = 0.0;
  @AutoLogOutput private boolean isOnTarget = false;

  private final ElevatorModuleIO moduleIO;
  private final DigitalInput homeSwitch;
  private final ProfiledPIDController pid;
  private boolean manualOverride = false;

  // Feedforward constants for smooth movement (must be tuned)
  private static final double kS = 0.1; // Static friction
  private static final double kG = 0.4; // Gravity compensation
  private static final double kV = 0.2; // Velocity gain
  private static final double kA = 0.05; // Acceleration gain
  private final ElevatorFeedforward feedforward = new ElevatorFeedforward(kS, kG, kV, kA);

  /**
   * Constructor for Elevator subsystem.
   *
   * @param moduleIO Hardware interface implementation for elevator motors.
   * @param homeSwitch Digital input limit switch for homing.
   */
  public Elevator(ElevatorModuleIO moduleIO, DigitalInput homeSwitch) {
    this.moduleIO = moduleIO;
    this.homeSwitch = homeSwitch;

    pid = new ProfiledPIDController(
        PElevator.proportional.getValue(),
        PElevator.integral.getValue(),
        PElevator.derivative.getValue(),
        new TrapezoidProfile.Constraints(
            PElevator.speedlimit.getValue(), 
            PElevator.accelerationLimit.getValue()
        )
    );

    // Start with the current height as the target
    targetHeight = moduleIO.getHeightMeters();
    pid.setGoal(targetHeight);
  }

  @Override
  public void periodic() {
    // Update sensor inputs
    moduleIO.updateInputs();
    currentHeight = moduleIO.getHeightMeters();

    // Reset encoder if home switch is pressed
    if (!homeSwitch.get()) {
      moduleIO.resetEncoder();
      currentHeight = 0.0;
      targetHeight = 0.0;
      pid.reset(0.0);
    }

    // If manual override is enabled, skip PID control
    if (manualOverride) return;

    // Clamp the target height to prevent exceeding limits
    double maxHeight = PElevator.MaxHeight.getValue();
    targetHeight = Math.max(0.0, Math.min(targetHeight, maxHeight));
    pid.setGoal(targetHeight);

    // Compute PID output and apply feedforward compensation
    double pidOutput = pid.calculate(currentHeight, targetHeight);
    double feedforwardOutput = feedforward.calculate(pid.getSetpoint().velocity);
    double finalOutput = pidOutput + feedforwardOutput;

    // Prevent downward movement if at home position
    if (finalOutput < 0 && !homeSwitch.get()) {
      finalOutput = 0;
    }

    // Engage brake if on target; otherwise, apply voltage
    if (Math.abs(currentHeight - targetHeight) < PElevator.tolerance.getValue()) {
      isOnTarget = true;
      moduleIO.setBraked(true);
      moduleIO.setVoltage(0);
    } else {
      isOnTarget = false;
      moduleIO.setBraked(false);
      moduleIO.setVoltage(finalOutput);
    }

    // SmartDashboard logging for debugging
    SmartDashboard.putNumber("Elevator Height", currentHeight);
    SmartDashboard.putNumber("Target Height", targetHeight);
    SmartDashboard.putBoolean("At Target", isOnTarget);
    SmartDashboard.putBoolean("Manual Override", manualOverride);
  }

  /**
   * Sets a new target height for the elevator (using PID control).
   *
   * @param height Desired height in meters.
   */
  public void setTargetHeight(double height) {
    targetHeight = Math.max(0.0, Math.min(height, PElevator.MaxHeight.getValue()));
    pid.setGoal(targetHeight);
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

  /** Holds the current position using PID. */
  public void holdPositionPID() {
    manualOverride = false;
    targetHeight = currentHeight;
    pid.setGoal(currentHeight);
  }

  /** Holds the current position using braking mode. */
  public void holdPositionBrake() {
    manualOverride = true;
    targetHeight = currentHeight;
    pid.setGoal(currentHeight);
  }

  /** Emergency stop function that immediately disables motor output. */
  public void emergencyStop() {
    moduleIO.setVoltage(0);
    manualOverride = true;
  }

   /**
   * @return The current elevator height in meters.
   */
  public double getCurrentHeight() { return currentHeight; }

  /**
   * @return True if the elevator is on target (within tolerance), false otherwise.
   */
  public boolean isOnTarget() { return isOnTarget; }
}







  
