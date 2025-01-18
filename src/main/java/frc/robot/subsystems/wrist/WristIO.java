package frc.robot.subsystems.wrist;

import org.littletonrobotics.junction.AutoLog;

public interface WristIO {
  @AutoLog
  public static class WristIOInputs {
    double wristLocationRad = 0.0;
    double wristSpeedRad = 0.0;

    double wristCurrentAmps = 0.0;
    double wristAppliedVoltage = 0.0;
  }

  // updates the given inputs with new values(advantage kit stuff)
  public default void updateInputs(WristIOInputsAutoLogged inputs) {}

  // sets the speed of the wrist to the amount
  public default void setSpeed(double speed) {}
  ;

  // stops the wrist at the given angle, locks it
  public default void stopWrist() {}

  // gets the current of the wrist in radians
  public default double getAngleRad() {
    return 0.0;
  }

  // gets the lowest possible angle of the wrist in radians
  public default double getLowestAngleRad() {
    return 0.0;
  }

  // gets the highest possible value of the wrist in radians
  public default double getHighestAngleRad() {
    return 0.0;
  }
}
