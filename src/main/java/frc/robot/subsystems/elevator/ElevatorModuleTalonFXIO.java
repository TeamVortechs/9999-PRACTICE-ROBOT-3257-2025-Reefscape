package frc.robot.subsystems.elevator;

import com.ctre.phoenix6.configs.MotorOutputConfigs;
import com.ctre.phoenix6.configs.TalonFXConfiguration;
import com.ctre.phoenix6.hardware.TalonFX;
import com.ctre.phoenix6.signals.NeutralModeValue;
import org.littletonrobotics.junction.AutoLogOutput;

/**
 * Implementation of ElevatorModuleIO using two TalonFX motor controllers. The two motors drive the
 * same gear but are mounted oppositely.
 */
public class ElevatorModuleTalonFXIO implements ElevatorModuleIO {
  private final TalonFX leftMotor;
  private final TalonFX rightMotor;

  @AutoLogOutput private double leftHeight;
  @AutoLogOutput private double rightHeight;

  /** Constructs the TalonFX-based elevator module. */
  public ElevatorModuleTalonFXIO(int motorIDLeft, int motorIDRight, String canbusName) {
    this.leftMotor = new TalonFX(motorIDLeft, canbusName);
    this.rightMotor = new TalonFX(motorIDRight, canbusName);

    TalonFXConfiguration leftConfig = new TalonFXConfiguration();
    TalonFXConfiguration rightConfig = new TalonFXConfiguration();

    // Configure motor inversions
    MotorOutputConfigs leftMotorOutput = new MotorOutputConfigs();
    // leftMotorOutput.Inverted = InvertedValue.CounterClockwise_Positive;

    MotorOutputConfigs rightMotorOutput = new MotorOutputConfigs();
    // rightMotorOutput.Inverted = InvertedValue.Clockwise_Positive;

    leftConfig.MotorOutput = leftMotorOutput;
    rightConfig.MotorOutput = rightMotorOutput;

    leftMotor.getConfigurator().apply(leftConfig);
    rightMotor.getConfigurator().apply(rightConfig);

    // Set both motors to Brake mode by default.
    leftMotor.setNeutralMode(NeutralModeValue.Brake);
    rightMotor.setNeutralMode(NeutralModeValue.Brake);
  }

  @Override
  public void updateInputs(ElevatorModuleIOInputsAutoLogged inputs) {
    inputs.elevatorMotor1CurrentHeightMeter = leftMotor.get();
    inputs.elevatorMotor1CurrentHeightMeter = getHeightMeters(0);

    inputs.elevatorMotor1CurrentAmps = leftMotor.getStatorCurrent().getValueAsDouble();
    inputs.elevatorMotor1AppliedVolts = leftMotor.getMotorVoltage().getValueAsDouble();

    inputs.elevatorMotor2CurrentHeightMeter = leftMotor.get();
    inputs.elevatorMotor2CurrentHeightMeter = getHeightMeters(0);

    inputs.elevatorMotor2CurrentAmps = leftMotor.getStatorCurrent().getValueAsDouble();
    inputs.elevatorMotor2AppliedVolts = leftMotor.getMotorVoltage().getValueAsDouble();

    inputs.isStalled = checkIfStalled();
  }

  /**
   * Returns the current elevator height in meters by averaging both motor encoders. this won't work
   * until we do the math with the gears to find out how much one rotation is in length
   */
  @Override
  public double getHeightMeters() {
    leftHeight = leftMotor.getPosition().getValueAsDouble();
    rightHeight = rightMotor.getPosition().getValueAsDouble();
    // System.out.println(
    //     "left height: "
    //         + leftHeight
    //         + " right height: "
    //         + rightHeight
    //         + " average: "
    //         + (leftHeight + rightHeight) / 2.0);
    return (leftHeight + rightHeight) / 2.0;
  }

  /** Sets the voltage to both motors. */
  @Override
  public void setVoltage(double volts) {
    leftMotor.setVoltage(volts);
    rightMotor.setVoltage(volts);

    System.out.println("setting voltage to " + volts);
  }

  /** Resets both motor encoders to zero. */
  @Override
  public void resetEncoders() {
    leftMotor.setPosition(0);
    rightMotor.setPosition(0);
  }

  /** Stops the motor immediately. */
  @Override
  public void stop() {
    leftMotor.stopMotor();
    rightMotor.stopMotor();
  }

  /** returns true if either motor has exceeded 40 amps of torque current currently nonfunctional */
  @Override
  public boolean checkIfStalled() {
    // return (Math.abs(leftMotor.getTorqueCurrent().getValueAsDouble()) > 40
    //     || Math.abs(rightMotor.getTorqueCurrent().getValueAsDouble()) > 40);
    return false;
  }

  /** Sets the speed of the motors (manual control mode). */
  @Override
  public void setSpeed(double speed) {
    System.out.println("ModuleIO receiving this speed: " + speed);
    leftMotor.set(speed);
    rightMotor.set(speed);
  }

  /** Sets the neutral mode for both motors (Brake or Coast). */
  @Override
  public void setBraked(boolean braked) {
    NeutralModeValue mode = braked ? NeutralModeValue.Brake : NeutralModeValue.Coast;
    leftMotor.setNeutralMode(mode);
    rightMotor.setNeutralMode(mode);
  }

  @Override
  public double getVoltage() {
    return leftMotor.getMotorVoltage().getValueAsDouble();
  }
}
