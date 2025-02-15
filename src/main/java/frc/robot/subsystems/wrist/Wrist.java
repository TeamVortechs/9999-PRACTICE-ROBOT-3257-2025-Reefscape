package frc.robot.subsystems.wrist;

import edu.wpi.first.math.controller.ProfiledPIDController;
import edu.wpi.first.math.trajectory.TrapezoidProfile;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.KDoublePreferences.PWrist;
import frc.robot.commands.wrist.SetWristRollerSpeedCommand;
import org.littletonrobotics.junction.AutoLogOutput;
import org.littletonrobotics.junction.Logger;

// again I'm not adding stuff to this class while we don't whats gonna go here
public class Wrist extends SubsystemBase {

  private WristIO wristIO;
  private WristIOInputsAutoLogged inputsAutoLogged = new WristIOInputsAutoLogged();

  private static double stageAngle = Math.toRadians(114.738651129);
  public static double intakeAngle = Math.toRadians(140);
  private static double Stage2angle = Math.toRadians(90);

  @AutoLogOutput private double CurrentAngle = 0;

  @AutoLogOutput private double targetAngle = 0;

  private final double targetBuffer = 0.05;

  @AutoLogOutput private boolean manualOverride = false;

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
    SmartDashboard.putNumber("Canrange distance", wristIO.getDistance());
    SmartDashboard.putBoolean("Canrange detected", wristIO.isDetected());

    wristIO.updateInputs(inputsAutoLogged);
    Logger.processInputs("Wrist", inputsAutoLogged);

    if (manualOverride) {
      System.out.println("MANUAL OVERRIDE WRIST ENGAGED");
      return;
    }

    CurrentAngle = wristIO.getAngleRad();
    double diffHeight = targetAngle - CurrentAngle;

    if (diffHeight < targetBuffer) return;

    double wristSpeed = PID.calculate(diffHeight);

    // individually move each elevator to that position

    // sets the speed of the wrist
    wristIO.setArmSpeed(wristSpeed);
  }

  public boolean isOnTarget() {
    return Math.abs(targetAngle - CurrentAngle) < targetBuffer;
  }
  // Math.abs(targetAngle - CurrentAngle) > 0.1 ||

  public void setBraked(boolean braked) {
    wristIO.setBraked(braked);
  }

  public double getAngleRad() {
    return wristIO.getAngleRad();
  }

  // public boolean isDetected() {
  //   return wristIO.isDetected();
  // }

  public void setManualSpeed(double speed) {
    manualOverride = true;
    wristIO.setArmSpeed(speed);
  }

  public void setTargetAngle(double angle) {
    manualOverride = false;
    this.targetAngle = angle;
  }

  public double getTargetAngle() {
    return targetAngle;
  }

  public void setRollerSpeed(double speed) {
    wristIO.setRollerSpeed(speed);
  }

  public double getCanDistance() {
    return wristIO.getDistance();
  }

  // public boolean isCanDetected() {
  //   return wristIO.isDetected();
  // }

  public boolean isCanCloserThan(double distance) {
    return getCanDistance() < distance;
  }

  // commands
  public Command rollWristUntilDetectedCommand(double speed, double distanceModifier) {
    return new SetWristRollerSpeedCommand(this, speed)
        .unless(() -> isCanCloserThan(distanceModifier));
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
