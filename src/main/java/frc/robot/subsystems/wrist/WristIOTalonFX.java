package frc.robot.subsystems.wrist;

import com.ctre.phoenix6.hardware.CANrange;
import com.ctre.phoenix6.hardware.TalonFX;
import com.ctre.phoenix6.signals.NeutralModeValue;

import edu.wpi.first.wpilibj.Encoder;
import edu.wpi.first.wpilibj.motorcontrol.PWMSparkMax;

public class WristIOTalonFX implements WristIO {
  private TalonFX arm = null;
  private PWMSparkMax rollers = null;
  private Encoder armEncoder;
  private CANrange Canrange;

  public WristIOTalonFX(
      int armID, int rollerID, String canbusName, int armEncoderID, int CanrangeID) {

    this.arm = new TalonFX(armID, canbusName);
    this.rollers = new PWMSparkMax(rollerID);
    this.armEncoder = new Encoder(armEncoderID, armEncoderID);
    this.Canrange = new CANrange(CanrangeID);
  }

  @Override
  public void setArmSpeed(double speed) {
    // arm.set(speed);
  }

  public void setRollerSpeed(double speed) {
    rollers.set(speed);
  }

  @Override
  public double getAngleRad() {
    return armEncoder.get() * (90 / 500);
  }

  @Override
  public void updateInputs(WristIOInputsAutoLogged inputs) {
    inputs.wristAppliedVoltage = arm.getMotorVoltage().getValueAsDouble();
    inputs.wristCurrentAmps = arm.getStatorCurrent().getValueAsDouble();
    inputs.wristSpeedRad = arm.get();
    inputs.wristLocationRad = armEncoder.get() * (90 / 500);
  }

  @Override
  public void setBraked(boolean braked) {
    if (braked) {
      arm.setNeutralMode(NeutralModeValue.Brake);
    } else {
      arm.setNeutralMode(NeutralModeValue.Coast);
    }
  }

  @Override
  public boolean isDetected() {
    return Canrange.getIsDetected().getValue();
  }

  @Override
  public double getDistance() {
    return Canrange.getDistance().getValueAsDouble();
  }
}
