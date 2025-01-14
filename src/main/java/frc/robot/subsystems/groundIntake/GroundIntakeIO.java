package frc.robot.subsystems.groundIntake;

import org.littletonrobotics.junction.AutoLog;

public interface GroundIntakeIO {

  @AutoLog
  public static class GroundIntakeIOInputs {
    double wristEncoder = 0;
    double wristMotorSpeed = 0;
    double wristAppliedVoltage = 0;
    double wristCurrentAmps = 0;

    double intakeMotorSpeed;
    double intakeAppliedVoltage = 0;
    double intakeCurrentAmps = 0;
  }

  public default double getMaxWristHeight() {
    return 0;
  }

  public default double getMinWristHeight() {
    return 0;
  }

  public default void setWristSpeed(double speed) {}

  public default void setIntakeSpeed(double speed) {}
}
