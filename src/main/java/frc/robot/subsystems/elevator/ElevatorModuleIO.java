package frc.robot.subsystems.elevator;

import org.littletonrobotics.junction.AutoLog;

// this is just a prototype, will be more fleshed out later.
public interface ElevatorModuleIO {
  @AutoLog
  public static class ElevatorModuleIOInputs {
    double elevatorCurrentHeightMeter = 0.0;

    double elevatorCurrentAmps = 0.0;
    double elevatorAppliedVolts = 0.0;
  }

  public default void updateInputs(ElevatorModuleIOInputs inputs) {}

  // sets the elevator height to the given number
  public default void setHeight(double height) {}

  // reminder for myself to use the given PID stuff
}
