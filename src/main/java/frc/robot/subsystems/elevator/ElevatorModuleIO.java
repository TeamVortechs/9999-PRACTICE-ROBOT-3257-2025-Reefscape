package frc.robot.subsystems.elevator;

import org.littletonrobotics.junction.AutoLog;

/**
 * Interface for Elevator Module IO implementations. This abstracts all hardware interactions for
 * the elevator.
 */
public interface ElevatorModuleIO {
  @AutoLog
  public static class ElevatorModuleIOInputs {
    double elevatorMotor1CurrentHeightMeter = 0;
    double elevatorMotor1CurrentSpeedMeter = 0;

    double elevatorMotor1CurrentAmps = 0;
    double elevatorMotor1AppliedVolts = 0;

    double elevatorMotor2CurrentAmps = 0;
    double elevatorMotor2AppliedVolts = 0;

    double elevatorMotor2CurrentSpeedMeter = 0;
    double elevatorMotor2CurrentHeightMeter = 0;
  }

  // updates the given inputs with new values(advantage kit stuff)
  public default void updateInputs() {}

  // sets the elevator height to the given number
  public default void setSpeed(double Speed) {}

  /** Stops the motor immediately */
  void stop();

  // gets the highest possible height of the elevator in radians
  public default double getMaxHeight() {
    return 0.0;
  }

  public default void setVoltage(double volt) {}

  public default void resetEncoder() {}

  public default void setBraked(boolean braked) {}
  // gets the height of the elevator in meters
  public default double getHeightMeters() {
    // CANrange distance = new CANrange(0);
    // double CurrentHeight = distance.getDistance().getValueAsDouble();

    if (Math.abs(getHeightMeters(0) - getHeightMeters(1)) > 0.1) {
      System.out.println(
          "PROBLEM!!!! GET HEIGHT OF ELEVATORS IS RETURNING SMTN DIFFERENT! (ELEVATOR MODULE)");
      System.out.println(getHeightMeters(0));
      System.out.println(getHeightMeters(1));
    }

    return (getHeightMeters(0) + getHeightMeters(1)) / 2;
  }

  public default double getHeightMeters(int motor) {
    return 0;
  }

  public default boolean isMaxHeight() {
    return false;
  }
  // public default void setPostion(double position){
  // }
  // reminder for myself to use the given PID stuff
}
