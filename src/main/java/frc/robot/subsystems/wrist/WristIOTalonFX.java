package frc.robot.subsystems.wrist;

import com.ctre.phoenix6.hardware.TalonFX;

public class WristIOTalonFX implements WristIO {
  private TalonFX arm = new TalonFX(14);

  @Override
  public void setSpeed(double speed) {
    arm.set(speed);
  }
}
