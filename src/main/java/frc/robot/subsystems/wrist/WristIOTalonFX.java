package frc.robot.subsystems.wrist;

import com.ctre.phoenix6.hardware.TalonFX;
import com.ctre.phoenix6.signals.NeutralModeValue;

public class WristIOTalonFX implements WristIO {
  private TalonFX arm = new TalonFX(14);

  @Override
  public void setSpeed(double speed) {
    arm.set(speed);
  }

  @Override
  public void updateInputs(WristIOInputsAutoLogged inputs) {
    inputs.wristAppliedVoltage = arm.getMotorVoltage().getValueAsDouble();
    inputs.wristCurrentAmps = arm.getStatorCurrent().getValueAsDouble();
    inputs.wristSpeedRad = arm.get();
    inputs.wristLocationRad = 0;
  }

  @Override
  public void setBraked(boolean braked) {
    if (braked) {
      arm.setNeutralMode(NeutralModeValue.Brake);
    } else {
      arm.setNeutralMode(NeutralModeValue.Coast);
    }
  }

  /*
  *     double wristLocationRad = 0.0;
   double wristSpeedRad = 0.0;

   double wristCurrentAmps = 0.0;
   double wristAppliedVoltage = 0.0;
  */
}
