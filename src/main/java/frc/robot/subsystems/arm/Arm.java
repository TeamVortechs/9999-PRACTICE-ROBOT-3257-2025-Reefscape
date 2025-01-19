package frc.robot.subsystems.arm;

import edu.wpi.first.math.controller.ProfiledPIDController;
import edu.wpi.first.math.interpolation.InterpolatingDoubleTreeMap;
import edu.wpi.first.math.trajectory.TrapezoidProfile;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.KDoublePreferences.PArm;
import org.littletonrobotics.junction.AutoLogOutput;
import org.littletonrobotics.junction.Logger;

// again I'm not adding stuff to this class while we don't whats gonna go here
public class Arm extends SubsystemBase {

  // think of this like the direct hardware interface, it's just like a talon interface that we code
  // ourselves
  private ArmIO armIO;

  // all of the logged inputs from the armIO
  private ArmIOInputsAutoLogged inputsAutoLogged = new ArmIOInputsAutoLogged();

  // classes for PID control
  // this goes to -10 when we don't want the arm to move anywhere
  @AutoLogOutput private double targetAngleRad = -10;

  // this is just outside of the method so AK can log it
  @AutoLogOutput private double wantedSpeed;

  // this is how we find the downward force on the motor at an specific point of the arm(not used rn
  // bc we don't have the air pistons and don't even know if we need it)
  private InterpolatingDoubleTreeMap weightMap = new InterpolatingDoubleTreeMap();

  // this is just a helper class that calculates the PID speed
  private ProfiledPIDController PID;

  public Arm(ArmIO armIO) {
    // the hardware interface that the arm uses to the specific one given by the creator of this
    // class(in other words allows switching between motors and advantage kit compabipility)
    this.armIO = armIO;

    // builds the PID tuner for the first time
    rebuildPIDtuner();

    // builds the weight map here with given values (is commented out so it doesn't do anything)
    // weightMap.put(0d, 0d);
  }

  // logs Advantage kit and does PDI logic
  @Override
  public void periodic() {
    // advantageKit inputs updating
    armIO.updateInputs(inputsAutoLogged);
    Logger.processInputs("Wrist", inputsAutoLogged);

    // this means we don't want to set the arm angle bc it hasn't been set for the first time yet
    if (targetAngleRad == -10) {
      return;
    }

    // calculates the wanted rotation from the PID and sets the motor to that position
    double curAngle = getAngleRad();
    double angleDiff = targetAngleRad - curAngle;
    wantedSpeed = PID.calculate(angleDiff);

    // finds the downward force from the arm weight and add that
    // this is commented out bc we don't have the weight map and don't know if we need this
    // double weightOffset = weightMap.get(curAngle);
    // wantedSpeed += weightOffset;

    // this is commented out while we don't have an encoder
    // armIO.setSpeed(wantedSpeed);

    // stops the arm if it outside of the target bounds
    // armIO.speedCheck();
  }

  // sets the wether or not the arm has "brakes" on it. If this is true the motor idle mode will be
  // set to stop. I reccomend keeping this off true for now bc it doesn't work and idk if it breaks
  // the motor
  public void setBraked(boolean braked) {
    armIO.setBraked(braked);
  }

  // gets the angle in radians of the motor
  public double getAngleRad() {
    return armIO.getAngleRad();
  }

  // sets the angle that the PID loop attempts to go to
  public double getTargetAngleRad() {
    return targetAngleRad;
  }

  // sets the target angle, or the angle that the PID loop will try to path to
  public void setTargetAngleRad(double targetAngle) {

    // stops the target angle from going above the appropriate value
    if (targetAngle < armIO.getLowestAngleRad()) {
      targetAngle = armIO.getLowestAngleRad();
    }

    if (targetAngle > armIO.getHighestAngleRad()) {
      targetAngle = armIO.getHighestAngleRad();
    }

    this.targetAngleRad = targetAngle;
  }

  // sets the speed of the motor, this will most likely be deprecated when we get the encoder and
  // can do set point
  public void setSpeed(double speed) {
    armIO.setSpeed(speed);
  }

  // rebuilds the pid tuner, this is only good while we test PID constants so we cna quickly switch
  // with preferences without having to redeploy
  public void rebuildPIDtuner() {
    // builds the PID tuner. It seems complicated but really it's just building a new one with all
    // the values form preferences
    PID =
        new ProfiledPIDController(
            PArm.proportional.getValue(),
            PArm.integral.getValue(),
            PArm.derivative.getValue(),
            new TrapezoidProfile.Constraints(
                PArm.speedLimit.getValue(), PArm.accelerationLimit.getValue()));
  }
}
