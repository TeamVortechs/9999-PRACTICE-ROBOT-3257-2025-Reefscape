package frc.robot.subsystems.wrist;

import com.ctre.phoenix6.hardware.TalonFX;
import com.ctre.phoenix6.signals.NeutralModeValue;

public class WristIOTalonFX implements WristIO {
  private TalonFX arm = new TalonFX(14);
  private TalonFX rollers = new TalonFX(5);

  @Override
  public void setArmSpeed(double speed) {
    arm.set(speed);
  }
  public void setRollerSpeed(double speed){
    rollers.set(speed);
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
