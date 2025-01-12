package frc.robot.subsystems.groundIntake;

public class GroundIntake {
  public void moveToIntakePos() {}

  public void powerIntake() {}

  public void runIntake() {}

  public GroundIntakeState getState() {
    return GroundIntakeState.DORMANT;
  }

  public enum GroundIntakeState {
    DORMANT,
    TRAVELLING,
    INTAKING;
  }
}
