package frc.robot.subsystems.wrist;

import edu.wpi.first.wpilibj.Encoder;
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
  public default void setArmSpeed(double speed) {}

  public default void setRollerSpeed(double speed) {}

  // stops the wrist at the given angle, locks it
  public default void stopWrist() {}

  // gets the current of the wrist in radians
  public default double getAngleRad() {
    Encoder encoder = new Encoder(0, 0);
    double SensorPosition = Math.toRadians(encoder.get() * 90 / 500);
    return SensorPosition;
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
