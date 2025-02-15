package frc.robot.subsystems.wrist;

import com.ctre.phoenix6.hardware.CANrange;
import com.ctre.phoenix6.hardware.TalonFX;
import com.ctre.phoenix6.signals.NeutralModeValue;
import com.revrobotics.spark.SparkLowLevel.MotorType;
import com.revrobotics.spark.SparkMax;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;

public class WristIOTalonFX implements WristIO {
  private TalonFX arm = null;
  private SparkMax rollers = null;
  private CANrange Canrange;

  public WristIOTalonFX(int armID, int rollerID, String canbusName, int CanrangeID) {

    this.arm = new TalonFX(armID, canbusName);
    this.rollers = new SparkMax(rollerID, MotorType.kBrushless);
    this.Canrange = new CANrange(CanrangeID);
  }

  @Override
  public void setArmSpeed(double speed) {
    arm.set(speed);
  }

  public void setRollerSpeed(double speed) {
    rollers.set(speed);
  }

  @Override
  public double getAngleRad() {
    double angle = arm.getPosition().getValue().baseUnitMagnitude() * 2 * Math.PI;

    SmartDashboard.putNumber("measured angle", angle);

    return angle;
  }

  @Override
  public void updateInputs(WristIOInputsAutoLogged inputs) {
    inputs.wristAppliedVoltage = arm.getMotorVoltage().getValueAsDouble();
    inputs.wristCurrentAmps = arm.getStatorCurrent().getValueAsDouble();
    inputs.wristSpeedRad = arm.get();
    inputs.wristLocationRad = getAngleRad();
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
