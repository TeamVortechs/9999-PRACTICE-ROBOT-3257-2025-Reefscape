package frc.robot.subsystems.elevator;

/**
 * Interface for Elevator Module IO implementations. This abstracts all hardware interactions for
 * the elevator.
 */
public interface ElevatorModuleIO {
  /** Update any sensor inputs if needed. */
  default void updateInputs() {}

  /**
   * Returns the current elevator height (in meters).
   *
   * @return Elevator height.
   */
  double getHeightMeters();

  /**
   * Sets the voltage to the elevator motors.
   *
   * @param volts Voltage command.
   */
  void setVoltage(double volts);

  /** Resets the encoders (e.g., to zero at home). */
  void resetEncoder();

  /**
   * Sets the neutral mode for the motors.
   *
   * @param braked If true, motors will be in Brake mode; if false, in Coast mode.
   */
  default void setBraked(boolean braked) {}
}
