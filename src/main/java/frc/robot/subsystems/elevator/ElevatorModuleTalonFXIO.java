package frc.robot.subsystems.elevator;

import com.ctre.phoenix6.hardware.TalonFX;
import com.ctre.phoenix6.signals.NeutralModeValue;
import frc.robot.Constants;
import frc.robot.KDoublePreferences.PElevator;

public class ElevatorModuleTalonFXIO implements ElevatorModuleIO {
  private TalonFX elevatorMotorLeft = new TalonFX(Constants.ELEVATOR_MOTOR_LEFT_ID, "Canivore");
  private TalonFX elevatorMotorRight = new TalonFX(Constants.ELEVATOR_MOTOR_RIGHT_ID, "Canivore");
  // private Encoder encoder = new Encoder(1, 1);

  public void setSpeed(double speed) {
    elevatorMotorLeft.set(speed);
    elevatorMotorRight.set(speed);
  }

  @Override
  public void updateInputs(ElevatorModuleIOInputsAutoLogged inputs) {
    inputs.elevatorMotor1CurrentHeightMeter = getHeightMeters(0);
    inputs.elevatorMotor1CurrentSpeedMeter = elevatorMotorLeft.getVelocity().getValueAsDouble();

    inputs.elevatorMotor1CurrentAmps = elevatorMotorLeft.getStatorCurrent().getValueAsDouble();
    inputs.elevatorMotor1AppliedVolts = elevatorMotorLeft.getMotorVoltage().getValueAsDouble();

    inputs.elevatorMotor2CurrentAmps = elevatorMotorRight.getStatorCurrent().getValueAsDouble();
    inputs.elevatorMotor2AppliedVolts = elevatorMotorRight.getMotorVoltage().getValueAsDouble();

    inputs.elevatorMotor2CurrentSpeedMeter = elevatorMotorRight.getVelocity().getValueAsDouble();
    inputs.elevatorMotor2CurrentHeightMeter = getHeightMeters(1);
  }

  @Override
  public double getHeightMeters(int motor) {
    return elevatorMotorLeft.getPosition().getValueAsDouble();
  }

  @Override
  public void setBraked(boolean braked) {
    if (braked) {
      elevatorMotorLeft.setNeutralMode(NeutralModeValue.Brake);
      elevatorMotorRight.setNeutralMode(NeutralModeValue.Brake);
    } else {
      elevatorMotorLeft.setNeutralMode(NeutralModeValue.Coast);
      elevatorMotorLeft.setNeutralMode(NeutralModeValue.Coast);
    }
  }

  @Override
  public boolean isMaxHeight() {
    if ((PElevator.MaxHeight.getValue() - getHeightMeters()) < 0.1) {
      return false;
    } else {
      setBraked(true);
      return true;
    }
  }

  @Override
  public void setVoltage(double volt) {
    elevatorMotorLeft.setVoltage(volt);
    elevatorMotorRight.setVoltage(volt);
  }

  @Override
  public void resetEncoder() {
    elevatorMotorLeft.setPosition(0);
    elevatorMotorRight.setPosition(0);
  }
}
