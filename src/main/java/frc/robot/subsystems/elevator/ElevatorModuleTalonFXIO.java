package frc.robot.subsystems.elevator;

import com.ctre.phoenix6.configs.MotorOutputConfigs;
import com.ctre.phoenix6.configs.TalonFXConfiguration;
import com.ctre.phoenix6.hardware.TalonFX;
import com.ctre.phoenix6.signals.InvertedValue;
import com.ctre.phoenix6.signals.NeutralModeValue;

/**
 * Implementation of ElevatorModuleIO using two TalonFX motor controllers. The two motors drive the
 * same gear but are mounted oppositely.
 */
public class ElevatorModuleTalonFXIO implements ElevatorModuleIO {
  private final TalonFX leftMotor;
  private final TalonFX rightMotor;

  /** Constructs the TalonFX-based elevator module. */
  public ElevatorModuleTalonFXIO(int motorIDLeft, int motorIDRight, String canbusName) {
    this.leftMotor = new TalonFX(motorIDLeft, canbusName);
    this.rightMotor = new TalonFX(motorIDRight, canbusName);

    TalonFXConfiguration leftConfig = new TalonFXConfiguration();
    TalonFXConfiguration rightConfig = new TalonFXConfiguration();

    // Configure motor inversions
    MotorOutputConfigs leftMotorOutput = new MotorOutputConfigs();
    leftMotorOutput.Inverted = InvertedValue.CounterClockwise_Positive;

    MotorOutputConfigs rightMotorOutput = new MotorOutputConfigs();
    rightMotorOutput.Inverted = InvertedValue.Clockwise_Positive;

    leftConfig.MotorOutput = leftMotorOutput;
    rightConfig.MotorOutput = rightMotorOutput;

    leftMotor.getConfigurator().apply(leftConfig);
    rightMotor.getConfigurator().apply(rightConfig);

    // Set both motors to Brake mode by default.
    leftMotor.setNeutralMode(NeutralModeValue.Brake);
    rightMotor.setNeutralMode(NeutralModeValue.Brake);
  }

  /** Returns the current elevator height in meters by averaging both motor encoders. */
  @Override
  public double getHeightMeters() {
    double leftHeight = leftMotor.getPosition().getValueAsDouble();
    double rightHeight = rightMotor.getPosition().getValueAsDouble();
    return (leftHeight + rightHeight) / 2.0;
  }

  /** Sets the voltage to both motors. */
  @Override
  public void setVoltage(double volts) {
    leftMotor.setVoltage(volts);
    rightMotor.setVoltage(volts);
  }

  /** Resets both motor encoders to zero. */
  @Override
  public void resetEncoder() {
    leftMotor.setPosition(0);
    rightMotor.setPosition(0);
  }

  /** Stops the motor immediately. */
  @Override
  public void stop() {
    leftMotor.stopMotor();
    rightMotor.stopMotor();
  }

  /** Sets the speed of the motors (manual control mode). */
  @Override
  public void setSpeed(double speed) {
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
}
