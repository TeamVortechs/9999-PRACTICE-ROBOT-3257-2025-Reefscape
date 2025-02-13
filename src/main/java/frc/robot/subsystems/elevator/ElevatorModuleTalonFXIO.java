package frc.robot.subsystems.elevator;

import com.ctre.phoenix6.configs.MotorOutputConfigs;
import com.ctre.phoenix6.configs.TalonFXConfiguration;
import com.ctre.phoenix6.hardware.TalonFX;
import com.ctre.phoenix6.signals.InvertedValue;
import com.ctre.phoenix6.signals.NeutralModeValue;
import frc.robot.Constants;

/**
 * Implementation of ElevatorModuleIO using two TalonFX motor controllers. The two motors drive the
 * same gear but are mounted oppositely.
 */
public class ElevatorModuleTalonFXIO implements ElevatorModuleIO {
  private final TalonFX leftMotor;
  private final TalonFX rightMotor;

  /** Constructs the TalonFX-based elevator module. */
  public ElevatorModuleTalonFXIO() {
    // Initialize motors using their CAN IDs (set in Constants).
    leftMotor = new TalonFX(Constants.ELEVATOR_MOTOR_LEFT_ID, "Canivore");
    rightMotor = new TalonFX(Constants.ELEVATOR_MOTOR_RIGHT_ID, "Canivore");

    // Create configuration objects for each motor.
    TalonFXConfiguration leftConfig = new TalonFXConfiguration();
    TalonFXConfiguration rightConfig = new TalonFXConfiguration();

    // Configure inversion using MotorOutputConfigs.
    MotorOutputConfigs leftMotorOutput = new MotorOutputConfigs();
    // Left motor runs normally.
    leftMotorOutput.Inverted = InvertedValue.CounterClockwise_Positive;

    MotorOutputConfigs rightMotorOutput = new MotorOutputConfigs();
    // Right motor is inverted because it is mounted oppositely.
    rightMotorOutput.Inverted = InvertedValue.Clockwise_Positive;

    // Apply the inversion settings to the configurations.
    leftConfig.MotorOutput = leftMotorOutput;
    rightConfig.MotorOutput = rightMotorOutput;

    // Apply the configurations to the motors.
    leftMotor.getConfigurator().apply(leftConfig);
    rightMotor.getConfigurator().apply(rightConfig);

    // Set both motors to Brake mode by default.
    leftMotor.setNeutralMode(NeutralModeValue.Brake);
    rightMotor.setNeutralMode(NeutralModeValue.Brake);
  }

  /**
   * Returns the current elevator height in meters by averaging both motor encoders.
   *
   * @return Elevator height.
   */
  @Override
  public double getHeightMeters() {
    double leftHeight = leftMotor.getPosition().getValueAsDouble();
    double rightHeight = rightMotor.getPosition().getValueAsDouble();
    return (leftHeight + rightHeight) / 2.0;
  }

  /**
   * Sets the voltage to both motors.
   *
   * @param volts Voltage command.
   */
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

  /**
   * Sets the neutral mode for both motors.
   *
   * @param braked True for Brake mode; false for Coast mode.
   */
  @Override
  public void setBraked(boolean braked) {
    NeutralModeValue mode = braked ? NeutralModeValue.Brake : NeutralModeValue.Coast;
    leftMotor.setNeutralMode(mode);
    rightMotor.setNeutralMode(mode);
  }
}
