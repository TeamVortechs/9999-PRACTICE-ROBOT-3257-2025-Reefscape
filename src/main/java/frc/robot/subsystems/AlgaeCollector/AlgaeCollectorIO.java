package frc.robot.subsystems.AlgaeCollector;

import org.littletonrobotics.junction.AutoLog;

public class AlgaeCollectorIO {
  @AutoLog
  public static class AlgaeCollectorIOInputs {
    double wristSpeedRad = 0.0;
    double wristPosRad = 0.0;

    double wristCurrentAmps = 0.0;
    double wristAppliedVolts = 0.0;

    double intakeSpeedRad = 0.0;
    double intakeAppliedVolts;
  }

  // not gonna do the methods here just yet
}

/*
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

  //updates the given inputs with new values(advantage kit stuff)
  public default void updateInputs(ElevatorModuleIOInputs inputs) {}

  // sets the elevator height to the given number
  public default void setSpeed(double height) {}

  //gets the highest possible height of the elevator in radians
  public default double getMaxHeight() {return 0.0;}

  //gets the height of the elevator in meters
  public default double getHeightMeters() {return 0.0;}

  // reminder for myself to use the given PID stuff
}

 */
