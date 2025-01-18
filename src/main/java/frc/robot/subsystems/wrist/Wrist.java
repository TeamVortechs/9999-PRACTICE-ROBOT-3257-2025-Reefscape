package frc.robot.subsystems.wrist;

// again I'm not adding stuff to this class while we don't whats gonna go here
public class Wrist {

  private WristIO wristIO;

  public Wrist(WristIO wristIO) {
    this.wristIO = wristIO;
  }

  public double getAngle() {
    return 0;
  }

  public double setAngle() {
    return 0;
  }

  public double targetAngle() {
    return 0;
  }

  public void setSpeed(double speed) {
    wristIO.setSpeed(speed);
  }
}
