package frc.robot.subsystems.elevator;

// I PROBABLY WILL WANT TO REFACTOR ALL OF THIS CODE INTO DIFFERENT CODE FOR ELEVATOR MODULE
public class Elevator {

  // sets the heihgt of the elevator using the pid system
  public void setTargetHeight(double height) {}

  // sets the height of the elevator but uses an enum to make it more expandable
  public void setHeight(ElevatorLevel level) {
    setTargetHeight(level.getHeight());
  }

  public double getHeight() {
    return 0;
  }

  public double getTargetHeight() {
    return 0;
  }

  // enum for each level that the elevator could be
  public enum ElevatorLevel {
    FIRST_LEVEL(1.0),
    SECOND_LEVEL(2.0),
    THIRD_LEVEL(3.0);

    private double height;

    private ElevatorLevel(double height) {
      this.height = height;
    }

    public double getHeight() {
      return height;
    }
  }
}
