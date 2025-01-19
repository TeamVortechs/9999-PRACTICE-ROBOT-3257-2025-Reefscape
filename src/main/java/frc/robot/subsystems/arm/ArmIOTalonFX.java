package frc.robot.subsystems.arm;

import com.ctre.phoenix6.hardware.TalonFX;
import com.ctre.phoenix6.signals.NeutralModeValue;

public class ArmIOTalonFX implements ArmIO {
  private TalonFX armMotor = new TalonFX(14);

  @Override
  public void setSpeed(double speed) {
    armMotor.set(speed);

    // uncommenting when we add PID loop code
    // speedCheck();
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

  // if the arm is out of bounds makes sure it doens't move further then return true
  @Override
  public boolean speedCheck() {
    double angle = getAngleRad();
    double speed = armMotor.get();

    if (angle > getHighestAngleRad()) {
      if (speed > 0) {
        setSpeed(0);
      }

      return true;
    }

    if (angle < getLowestAngleRad()) {
      if (speed < 0) {
        setSpeed(0);
      }

      return true;
    }

    return false;
  }

  // all this code doesn't do anythin while we don't have an encoder
  // gets the current of the wrist in radians
  @Override
  public double getAngleRad() {
    return 0.0;
  }

  // gets the lowest possible angle of the wrist angle in radians
  @Override
  public double getLowestAngleRad() {
    return 0.0;
  }

  // gets the highest possible value of the wrist angle in radians
  @Override
  public double getHighestAngleRad() {
    // 90 deg
    return Math.PI / 2;
  }
}
