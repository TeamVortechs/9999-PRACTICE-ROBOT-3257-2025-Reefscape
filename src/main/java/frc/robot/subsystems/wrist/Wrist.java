package frc.robot.subsystems.wrist;

import edu.wpi.first.math.controller.ProfiledPIDController;
import edu.wpi.first.math.trajectory.TrapezoidProfile;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants;
import frc.robot.KDoublePreferences.PWrist;
import org.littletonrobotics.junction.AutoLogOutput;
import org.littletonrobotics.junction.Logger;

// again I'm not adding stuff to this class while we don't whats gonna go here
public class Wrist extends SubsystemBase {

  private WristIO wristIO;
  private WristIOInputsAutoLogged inputsAutoLogged = new WristIOInputsAutoLogged();

  // private static double stageAngle = Math.toRadians(114.738651129);
  // public static double intakeAngle = Math.toRadians(140);
  // private static double Stage2angle = Math.toRadians(90);

  @AutoLogOutput private double CurrentAngle = 0;

  @AutoLogOutput private double targetAngle = 0;

  @AutoLogOutput private double pidOutput;

  private final double targetBuffer = 0.1;

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

    CurrentAngle = wristIO.getAngleRotations();

    if (getAngleRotations() > Constants.Arm.WRIST_HIGHEST_ANGLE || getAngleRotations() < -0.2) {
      setManualSpeed(0);
      System.out.println("WRIST OUT OF BOUNDS");
    }

    if (manualOverride) {
      System.out.println("MANUAL OVERRIDE WRIST ENGAGED");
      return;
    }

    double diffHeight = targetAngle - CurrentAngle;

    if (diffHeight < targetBuffer) return;

    // set target position to 100 rotations
    wristIO.PIDVoltage(targetAngle);

    // Math.abs(pidOutput) > PWrist.speedLimit.getValue()
    //     ? Math.copySign(PWrist.speedLimit.getValue(), pidOutput) // change later
    //     : pidOutput;

    // double wristSpeed = PID.calculate(CurrentAngle, targetAngle);
    // individually move each elevator to that position

    // sets the speed of the wrist
  }

  // returns wether or not the wrist is on target
  public boolean isOnTarget() {
    return Math.abs(targetAngle - CurrentAngle) < targetBuffer;
  }
  // Math.abs(targetAngle - CurrentAngle) > 0.1 ||

  // sets wether or not the wrist is braked(NOTE: THIS DOES NOT ACTUALLY STOP THE WRIST)
  public void setBraked(boolean braked) {
    wristIO.setBraked(braked);
  }

  // gets the rotation of the arm
  public double getAngleRotations() {
    return wristIO.getAngleRotations();
  }

  // public boolean isDetected() {
  //   return wristIO.isDetected();
  // }

  // returns wether or not the arm is clear from the elevator
  public boolean isClearFromElevator() {
    return wristIO.getAngleRotations() > 2;
  }

  // turns manual override and sets the manual speeed
  public void setManualSpeed(double speed) {
    manualOverride = true;
    wristIO.setArmSpeed(speed);
  }

  // sets the target angle of the PID
  public void setTargetAngle(double angle) {
    manualOverride = false;
    this.targetAngle = angle;
  }

  // gets the angle that the PID is pathing to
  public double getTargetAngle() {
    return targetAngle;
  }

  // sets the roller speed
  public void setRollerSpeed(double speed) {
    wristIO.setRollerSpeed(speed);
  }

  // gets the distance of the can Range
  public double getCanDistance() {
    return wristIO.getDistance();
  }

  // resets the encoder of the wrist
  public void resetWristEncoder() {
    wristIO.zeroArmEncoder();
  }

  // public boolean isCanDetected() {
  //   return wristIO.isDetected();
  // }

  // returns wether or not the canRange is closer than the given distance
  public boolean isCanCloserThan(double distance) {
    return getCanDistance() < distance;
  }

  // commands
  // this does not work! that is why it is commented out
  // public Command rollWristUntilDetectedCommand(double speed, double distanceModifier) {
  //   return new SetWristRollerSpeedCommand(this, speed)
  //       .unless(() -> isCanCloserThan(distanceModifier));
  // }

  // enum for each level that the wrist could be
  public enum WristAngle {
    STAGE1_ANGLE(Constants.Arm.WRIST_STAGE_2_ANGLE);
    // STAGE2_ANGLE(Stage2angle),
    // INTAKE_ANGLE(Constant);

    private double angle;

    private WristAngle(double angleRad) {
      this.angle = angleRad;
    }

    public double getAngle() {
      return angle;
    }
  }
}
