package frc.robot.subsystems.wrist;

public class WristIOSimulation implements WristIO {
  private double wristLocationRotations = 0.0;
  private double wristSpeedRotations = 0.0;

  private double wristCurrentAmps = 0.0;
  private double wristAppliedVoltage = 0.0;

  private double canRangeDistance = 0.0;

  private double rollersCurrent = 0.0;
  private double rollersEncoder = 0.0;
  private double rollersSpeed = 0.0;
  
  @Override
  public boolean isDetected() {
    return false;
  }
  @Override
  public double getDistance() {
    return 0.0;
  }
  @Override
  // updates the given inputs with new values(advantage kit stuff)
  public void updateInputs(WristIOInputsAutoLogged inputs) {

  }
  @Override
  // sets the speed of the wrist to the amount
  public void setArmSpeed( double speed) {

  }
  @Override
  public void setArmVoltage( double voltage) {

  }
  @Override
  public void setRollerSpeed( double speed) {

  }
  @Override
  // stops the wrist at the given angle, locks it
  public void stopWrist() {

  }
  @Override
  public void PIDVoltage( double targetAngle) {

  }
  @Override
  // gets the current of the wrist in radians
  public double getAngleRotations() {
    return 0;
  }
  @Override
  public void setBraked(boolean braked) {

  }
  @Override
  // gets the lowest possible angle of the wrist in radians
  public double getLowestAngleRad() {
    return 0.0;
  }
  @Override
  // gets the highest possible value of the wrist in radians
  public double getHighestAngleRad() {
    return 0.0;
  }
  @Override
  public void zeroArmEncoder() {

  }
}
