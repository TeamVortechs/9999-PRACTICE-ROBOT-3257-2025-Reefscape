package frc.robot.subsystems.wrist;

import edu.wpi.first.math.controller.ProfiledPIDController;
import edu.wpi.first.math.trajectory.TrapezoidProfile;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.KDoublePreferences.PWrist;
import org.littletonrobotics.junction.Logger;

// again I'm not adding stuff to this class while we don't whats gonna go here
public class Wrist extends SubsystemBase {

  private WristIO wristIO;
  private WristIOInputsAutoLogged inputsAutoLogged = new WristIOInputsAutoLogged();

  private static double stageAngle = Math.toRadians(114.738651129);
  public static double intakeAngle = Math.toRadians(140);
  private static double Stage2angle = Math.toRadians(90);

  private double CurrentAngle = 0;

  private double targetAngle = 0;

  private ProfiledPIDController PID =
      new ProfiledPIDController(
          PWrist.proportional.getValue(),
          PWrist.integral.getValue(),
          PWrist.derivative.getValue(),
          new TrapezoidProfile.Constraints(
              PWrist.speedLimit.getValue(), PWrist.accelerationLimit.getValue()));

  public Wrist(WristIO wristIO) {
    this.wristIO = wristIO;
  }

  @Override
  public void periodic() {
    // advantageKit inputs updating
    wristIO.updateInputs(inputsAutoLogged);
    Logger.processInputs("Wrist", inputsAutoLogged);

    CurrentAngle = wristIO.getAngleRad();
    double diffHeight = Math.abs(targetAngle - CurrentAngle);

    if (diffHeight < 0.1) return;

    double wristSpeed = PID.calculate(diffHeight);

    // individually move each elevator to that position

    // sets the speed of the wrist
    wristIO.setArmSpeed(wristSpeed);
  }

  public boolean isOnTarget() {
    return Math.abs(targetAngle - CurrentAngle) < 0.1;
  }
  // Math.abs(targetAngle - CurrentAngle) > 0.1 ||

  public void setBraked(boolean braked) {
    wristIO.setBraked(braked);
  }

  public double getAngleRad() {
    return wristIO.getAngleRad();
  }

  public void setTargetAngle(double angle) {
    this.targetAngle = angle;
  }

  public double getTargetAngle() {
    return targetAngle;
  }

  public void setRollerSpeed(double speed) {
    wristIO.setRollerSpeed(speed);
  }

  // enum for each level that the wrist could be
  public enum WristAngle {
    STAGE1_ANGLE(stageAngle),
    STAGE2_ANGLE(Stage2angle),
    INTAKE_ANGLE(intakeAngle);

    private double angle;

    private WristAngle(double angleRad) {
      this.angle = angleRad;
    }

    public double getAngle() {
      return angle;
    }
  }
}
