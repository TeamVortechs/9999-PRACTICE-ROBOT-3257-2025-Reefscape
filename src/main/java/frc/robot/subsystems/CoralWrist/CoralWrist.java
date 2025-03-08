package frc.robot.subsystems.CoralWrist;

import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants;
import frc.robot.KDoublePreferences.PWrist;
import org.littletonrobotics.junction.AutoLogOutput;
import org.littletonrobotics.junction.Logger;

// again I'm not adding stuff to this class while we don't whats gonna go here
public class CoralWrist extends SubsystemBase {

  private CoralWristIO wristIO;
  private WristIOInputsAutoLogged inputsAutoLogged = new WristIOInputsAutoLogged();

  @AutoLogOutput private double CurrentAngle = 0;

  @AutoLogOutput private double targetAngle = 0;

  @AutoLogOutput private double pidOutput;

  private final double targetBuffer = PWrist.tolerance.getValue();

  @AutoLogOutput private boolean manualOverride = false;

  public CoralWrist(CoralWristIO wristIO) {
    this.wristIO = wristIO;
  }

  @Override
  public void periodic() {
    // advantageKit inputs updating

    wristIO.updateInputs(inputsAutoLogged);
    Logger.processInputs("Wrist", inputsAutoLogged);

    CurrentAngle = wristIO.getAngleRotations();

    // if (getAngleRotations() > Constants.Arm.WRIST_HIGHEST_ANGLE
    //     || getAngleRotations() < -targetBuffer) {
    //   setManualSpeed(-0.05);
    //   System.out.println("WRIST OUT OF BOUNDS");
    // }

    // if (manualOverride) {
    //   System.out.println("Wrist MANUAL OVERRIDE WRIST ENGAGED");
    //   return;
    // }
    // set target position to 100 rotations
    wristIO.PIDVoltage(targetAngle);
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
  /*  public boolean isClearFromElevator() {
    return wristIO.getAngleRotations() > Constants.Arm.SCORING_ANGLE - 0.1;
  }
    /* */

  // turns manual override and sets the manual speeed
  public void setManualSpeed(double speed) {
    manualOverride = true;

    if (Math.abs(speed) > PWrist.manualSpeedLimit.getValue())
      speed = Math.copySign(PWrist.manualSpeedLimit.getValue(), speed);
    System.out.println("Above speed limit; rate limiting WRIST speed.");
    wristIO.setArmSpeed(speed);
  }

  // sets the target angle of the PID
  public void setTargetAngle(double angle) {
    manualOverride = false;
    this.targetAngle = angle;
    // System.out.println("setting target angle");
  }

  // gets the angle that the PID is pathing to
  public double getTargetAngle() {
    return targetAngle;
  }

  // sets the roller speed
  public void setRollerSpeed(double speed) {
    wristIO.setRollerSpeed(speed);
  }

  // resets the encoder of the wrist
  public void resetWristEncoder() {
    wristIO.zeroArmEncoder();
  }

  // enum for each level that the wrist could be
  public enum CoralWristAngle {
    INTAKE_ANGLE(Constants.CoralArm.CORAL_GROUND_INTAKE_ANGLE),
    SCORING_ANGLE(Constants.CoralArm.WRIST_CORAL_SCORE);

    // STAGE2_ANGLE(Stage2angle),

    private double angle;

    private CoralWristAngle(double angleRot) {
      this.angle = angleRot;
    }

    public double getAngle() {
      return angle;
    }
  }
}
