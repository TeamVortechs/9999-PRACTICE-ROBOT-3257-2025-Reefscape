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

  public default void updateInputs(WristIOInputs inputs) {}

  // sets the speed of the wrist to the amount
  public default void setSpeed(double speed) {}
  ;

  // stops the wrist at the given angle
  public default void stopWrist() {}
}
