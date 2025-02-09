package frc.robot.subsystems.elevator;

import com.ctre.phoenix6.hardware.TalonFX;
import com.ctre.phoenix6.signals.NeutralModeValue;
import edu.wpi.first.wpilibj.Encoder;
import frc.robot.KDoublePreferences.PElevator;

public class ElevatorModuleSparkMaxIO implements ElevatorModuleIO {
  private TalonFX Elevatormotor = new TalonFX(5, "Canivore");
  private TalonFX Elevatormotor1 = new TalonFX(6, "Canivore");
  private Encoder encoder = new Encoder(1, 1);

  public void setSpeed(double speed) {
    Elevatormotor.set(speed);
    Elevatormotor1.set(speed);
  }

  @Override
  public void updateInputs(ElevatorModuleIOInputsAutoLogged inputs) {
    inputs.elevatorMotor1CurrentHeightMeter = getHeightMeters(0);
    inputs.elevatorMotor1CurrentSpeedMeter = Elevatormotor.getVelocity().getValueAsDouble();

    inputs.elevatorMotor1CurrentAmps = Elevatormotor.getStatorCurrent().getValueAsDouble();
    inputs.elevatorMotor1AppliedVolts = Elevatormotor.getMotorVoltage().getValueAsDouble();

    inputs.elevatorMotor2CurrentAmps = Elevatormotor1.getStatorCurrent().getValueAsDouble();
    inputs.elevatorMotor2AppliedVolts = Elevatormotor1.getMotorVoltage().getValueAsDouble();

    inputs.elevatorMotor2CurrentSpeedMeter = Elevatormotor1.getVelocity().getValueAsDouble();
    inputs.elevatorMotor2CurrentHeightMeter = getHeightMeters(1);
  }

  @Override
  public double getHeightMeters(int motor) {
    return Elevatormotor.getPosition().getValueAsDouble();
  }

  @Override
  public void setBraked(boolean braked) {
    if (braked) {
      Elevatormotor.setNeutralMode(NeutralModeValue.Brake);
      Elevatormotor1.setNeutralMode(NeutralModeValue.Brake);
    } else {
      Elevatormotor.setNeutralMode(NeutralModeValue.Coast);
      Elevatormotor.setNeutralMode(NeutralModeValue.Coast);
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
    Elevatormotor.setVoltage(volt);
    Elevatormotor1.setVoltage(volt);
  }

  @Override
  public void resetEncoder() {
    encoder.reset();
  }
}
