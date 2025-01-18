package frc.robot.subsystems.wrist;

import com.ctre.phoenix6.hardware.TalonFX;

public class WristIOTalonFX implements WristIO {
  private TalonFX arm = new TalonFX(14);

  @Override
  public void setSpeed(double speed) {
    arm.set(speed);
  }

  @Override
  public updateInputs(WristIOInputsAutoLogged inputs) {
    
  }
  /*
   *     double wristLocationRad = 0.0;
    double wristSpeedRad = 0.0;

    double wristCurrentAmps = 0.0;
    double wristAppliedVoltage = 0.0;
   */
}
