package frc.robot.subsystems.wrist;

import org.littletonrobotics.junction.AutoLog;

public interface ArmIO {
  @AutoLog
  public static class ArmIOInputs {
    double armLocationRad = 0.0;
    double armSpeedRad = 0.0;

    double armCurrentAmps = 0.0;
    double armAppliedVoltage = 0.0;
  }

  // updates the given inputs with new values(advantage kit stuff)
  public default void updateInputs(ArmIOInputsAutoLogged inputs) {}

  // sets the speed of the wrist to the amount
  public default void setSpeed(double speed) {}

  // gets the current of the wrist in radians
  public default double getAngleRad() {
    return 0.0;
  }

  public default void setBraked(boolean braked) {}

  // gets the lowest possible angle of the wrist in radians
  public default double getLowestAngleRad() {
    return 0.0;
  }

  // gets the highest possible value of the wrist in radians
  public default double getHighestAngleRad() {
    return 0.0;
  }
}
