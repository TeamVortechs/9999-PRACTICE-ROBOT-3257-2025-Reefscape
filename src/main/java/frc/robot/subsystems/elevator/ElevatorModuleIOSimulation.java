package frc.robot.subsystems.elevator;

import edu.wpi.first.math.MathUtil;
import edu.wpi.first.math.controller.ElevatorFeedforward;
import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.math.system.plant.DCMotor;
import edu.wpi.first.math.system.plant.LinearSystemId;
import edu.wpi.first.wpilibj.simulation.DCMotorSim;
import frc.robot.KDoublePreferences.PElevator;
import org.littletonrobotics.junction.AutoLogOutput;

/**
 * Implementation of ElevatorModuleIO using two TalonFX motor controllers. The two motors drive the
 * same gear but are mounted oppositely.
 */
public class ElevatorModuleIOSimulation implements ElevatorModuleIO {

  private final double speedDivider = 1;

  private final DCMotorSim elevatorMotorsSim;

  private PIDController elevatorPIDController =
      new PIDController(
          PElevator.kP.getValue() / speedDivider,
          PElevator.kI.getValue() / speedDivider,
          PElevator.kD.getValue() / speedDivider);
  private ElevatorFeedforward elevatorFeedforward =
      new ElevatorFeedforward(
          PElevator.kS.getValue() / speedDivider,
          PElevator.kG.getValue() / speedDivider,
          PElevator.kV.getValue() / speedDivider);

  private double targetVel = PElevator.speedLimit.getValue() / speedDivider;
  private double targetAccel = PElevator.accelerationLimit.getValue() / speedDivider;
  // private double targetJerk = PElevator.jerkLimit.getValue(); // not actually used. neat

  private final double kGearRatio = 1; // well we're literally just reading rotations so /shrug

  @AutoLogOutput private double leftHeight;
  @AutoLogOutput private double rightHeight;

  /** Constructs the TalonFX-based elevator simulation. */
  public ElevatorModuleIOSimulation() {
    this.elevatorMotorsSim =
        new DCMotorSim(
            LinearSystemId.createDCMotorSystem(DCMotor.getKrakenX60(2), 0.001, kGearRatio),
            DCMotor.getKrakenX60(2));
  }

  // advantage kit logging stuff(everything in here gets logged every tick)
  @Override
  public void updateInputs(ElevatorModuleIOInputsAutoLogged inputs) {

    inputs.elevatorMotor1CurrentHeightMeter = elevatorMotorsSim.getAngularPositionRotations();
    // inputs.elevatorMotor1CurrentHeightMeter = getHeightMeters(0);

    inputs.elevatorMotor1CurrentAmps = elevatorMotorsSim.getCurrentDrawAmps();
    inputs.elevatorMotor1AppliedVolts = elevatorMotorsSim.getInputVoltage();

    inputs.elevatorMotor2CurrentHeightMeter = elevatorMotorsSim.getAngularPositionRotations();
    // inputs.elevatorMotor2CurrentHeightMeter = getHeightMeters(0);

    inputs.elevatorMotor2CurrentAmps = elevatorMotorsSim.getCurrentDrawAmps();
    inputs.elevatorMotor2AppliedVolts = elevatorMotorsSim.getInputVoltage();

    inputs.isStalled = checkIfStalled();

    elevatorMotorsSim.update(0.02);
  }

  /**
   * Returns the current elevator height in meters by averaging both motor encoders. this won't work
   * until we do the math with the gears to find out how much one rotation is in length
   */
  @Override
  public double getHeightMeters() {

    return elevatorMotorsSim.getAngularPositionRotations();
  }

  /** Sets the voltage to both motors. */
  @Override
  public void setVoltage(double volts) {
    elevatorMotorsSim.setInputVoltage(MathUtil.clamp(volts, -12, 12));
  }

  @Override
  public void PIDVoltage(double targetAngle) {
    double currentAngle = elevatorMotorsSim.getAngularPositionRotations();
    setVoltage(
        elevatorPIDController.calculate(currentAngle, targetAngle)
            + elevatorFeedforward.calculate(targetVel, targetAccel));
  }

  /** Resets both motor encoders to zero. */
  @Override
  public void resetEncoders() {
    elevatorMotorsSim.setAngle(0);
  }

  /** Stops the motor immediately. */
  @Override
  public void stop() {
    elevatorMotorsSim.setAngularVelocity(0);
  }

  /** returns true if either motor has exceeded 40 amps of torque current currently nonfunctional */
  @Override
  public boolean checkIfStalled() {
    // return (Math.abs(leftMotor.getTorqueCurrent().getValueAsDouble()) > 40
    //     || Math.abs(rightMotor.getTorqueCurrent().getValueAsDouble()) > 40);
    return false;
  }

  /** Sets the speed of the motors (manual control mode). */
  // @Override
  // public void setSpeed(double speed) {
  //   // System.out.println("ModuleIO receiving this speed: " + speed);
  //   leftMotor.set(speed);
  //   rightMotor.set(speed);
  // }

  /** Sets the neutral mode for both motors (Brake or Coast). */
  // @Override
  // public void setBraked(boolean braked) {
  //   NeutralModeValue mode = braked ? NeutralModeValue.Brake : NeutralModeValue.Coast;
  //   leftMotor.setNeutralMode(mode);
  //   rightMotor.setNeutralMode(mode);
  // }

  @Override
  public double getVoltage() {
    return elevatorMotorsSim.getInputVoltage();
  }
}
