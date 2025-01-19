package frc.robot.subsystems.wrist;

import com.ctre.phoenix6.hardware.TalonFX;
import com.ctre.phoenix6.signals.NeutralModeValue;

public class ArmIOTalonFX implements ArmIO {
  private TalonFX armMotor = new TalonFX(14);

  @Override
  public void setSpeed(double speed) {
    armMotor.set(speed);
  }

  @Override
  public void updateInputs(ArmIOInputsAutoLogged inputs) {
    inputs.armAppliedVoltage = armMotor.getMotorVoltage().getValueAsDouble();
    inputs.armCurrentAmps = armMotor.getStatorCurrent().getValueAsDouble();
    inputs.armSpeedRad = armMotor.get();
    inputs.armLocationRad = 0;
  }

  @Override
  public void setBraked(boolean braked) {
    if (braked) {
      armMotor.setNeutralMode(NeutralModeValue.Brake);
    } else {
      armMotor.setNeutralMode(NeutralModeValue.Coast);
    }
  }

  // all this code doesn't do anythin while we don't have an encoder
  // gets the current of the wrist in radians
  @Override
  public double getAngleRad() {
    return 0.0;
  }

  // gets the lowest possible angle of the wrist in radians
  @Override
  public double getLowestAngleRad() {
    return 0.0;
  }

  // gets the highest possible value of the wrist in radians
  @Override
  public double getHighestAngleRad() {
    return 0.0;
  }
}
