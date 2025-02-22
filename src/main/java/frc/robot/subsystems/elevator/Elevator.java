package frc.robot.subsystems.elevator;

import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.KDoublePreferences.PElevator;
import frc.robot.subsystems.wrist.Wrist;
import org.littletonrobotics.junction.AutoLogOutput;
import org.littletonrobotics.junction.Logger;

/**
 * Elevator subsystem responsible for controlling the lifting mechanism. Uses PID control for
 * precise movement and prevents unsafe operation via limit switches and software constraints.
 */
public class Elevator extends SubsystemBase {

  ElevatorModuleIOInputsAutoLogged inputs = new ElevatorModuleIOInputsAutoLogged();

  @AutoLogOutput private double currentHeight = 0.0;
  @AutoLogOutput private double targetHeight = 0.0;
  @AutoLogOutput private boolean isOnTarget = false;

  @AutoLogOutput double maxHeight;
  // @AutoLogOutput double pidOutput;

  private final ElevatorModuleIO moduleIO;
  // private final DigitalInput homeSwitch;
  // private final ProfiledPIDController pid;
  @AutoLogOutput private boolean manualOverride = false;
  @AutoLogOutput private int loopCount = 0; // Counter to reduce SmartDashboard updates

  private Wrist wrist;

  @AutoLogOutput public boolean wristAngleValid = true;

  /**
   * Constructor for the Elevator subsystem.
   *
   * @param moduleIO Hardware interface for elevator motors.
   * @param homeSwitch Digital input limit switch for homing.
   */
  public Elevator(
      ElevatorModuleIO moduleIO
          // , DigitalInput homeSwitch
          ,
      Wrist wrist) {
    // setPreferences();
    this.moduleIO = moduleIO;
    // this.homeSwitch = homeSwitch;

    this.wrist = wrist;

    currentHeight = moduleIO.getHeightMeters();
    targetHeight = moduleIO.getHeightMeters();
    // pid.setGoal(targetHeight);
  }

  @Override
  public void periodic() {
    moduleIO.updateInputs(inputs);
    Logger.processInputs("Elevator", inputs);

    // check to see if the elevator is stalling; if so, then stop the motors and cancel the next
    // movement

    if (moduleIO.checkIfStalled()) {
      System.out.println("ELEVATOR HAS STALLED ");
      moduleIO.stop();
      return;
    }

    currentHeight = moduleIO.getHeightMeters();

    // Increment loop counter and update SmartDashboard every 5 cycles (~100ms)
    loopCount++;
    if (loopCount % 5 == 0) {
      SmartDashboard.putNumber("Elevator Height", currentHeight);
      SmartDashboard.putNumber("Target Height", targetHeight);
      SmartDashboard.putBoolean("At Target", isOnTarget);
      SmartDashboard.putBoolean("Manual Override", manualOverride);

      if (manualOverride) {
        // System.out.println("ELEVATOR IS IN MANUAL OVERRIDE");
        return;
      }
    }

    if (!wrist.isClearFromElevator()) {
      // System.out.println("ELEVATOR IS NOT MOVING! THE WRIST ANGLE IS NOT VALID");
      moduleIO.setSpeed(0);
      return;
    }

    if (manualOverride) {

      if (getCurrentHeight() < PElevator.MinHeight.getValue() - PElevator.tolerance.getValue()
          || getCurrentHeight() > PElevator.MaxHeight.getValue()) {
        System.out.println("ELEVATOR OUT OF BOUDNS");
        setManualSpeed(0);
      }
      return;
    }

    if (Math.abs(currentHeight - targetHeight) < PElevator.tolerance.getValue()) {
      isOnTarget = true;
    } else {
      isOnTarget = false;
      // Clamp target height to prevent exceeding limits
      maxHeight = PElevator.MaxHeight.getValue();
      targetHeight = Math.max(0.0, Math.min(targetHeight, maxHeight));

      moduleIO.PIDVoltage(targetHeight);
    }
  }

  /** Sets a new target height for the elevator using PID control. */
  public void setTargetHeight(double height) {
    manualOverride = false;
    targetHeight = Math.max(0.0, Math.min(height, PElevator.MaxHeight.getValue()));
  }

  /** Allows manual control of the elevator, bypassing PID. */
  public void setManualSpeed(double speed) {
    manualOverride = true;

    if (Math.abs(speed) > PElevator.speedlimit.getValue())
      speed = Math.copySign(PElevator.speedlimit.getValue(), speed);
    System.out.println("Above speed limit; rate limiting speed.");
    moduleIO.setSpeed(speed);
  }

  /** Holds the current position using PID control. */
  public void holdPositionPID() {
    manualOverride = false;
    if (Math.abs(targetHeight - currentHeight) > PElevator.tolerance.getValue()) {
      targetHeight = currentHeight;
      moduleIO.PIDVoltage(targetHeight);
    }
  }

  /** Holds the current position using braking mode. */
  public void holdPositionBrake() {
    System.out.println("Yep. I'm braking.");
    manualOverride = true;
    moduleIO.stop();
  }

  /** Emergency stop function that immediately disables motor output. */
  public void emergencyStop() {
    moduleIO.stop();
    manualOverride = true;
  }

  // gest the current height of the elevator motor
  public double getCurrentHeight() {
    return currentHeight;
  }

  // returns wether or not the elevaotr is on target
  public boolean isOnTarget() {
    return isOnTarget;
  }

  /** resets encoders to read 0 and resets PID (setting it to begin at current height) */
  public void resetEncoders() {
    moduleIO.resetEncoders();
  }
}
