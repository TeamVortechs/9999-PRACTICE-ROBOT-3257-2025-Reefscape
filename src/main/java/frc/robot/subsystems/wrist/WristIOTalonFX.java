package frc.robot.subsystems.wrist;

import com.ctre.phoenix6.hardware.CANrange;
import com.ctre.phoenix6.hardware.TalonFX;
import com.ctre.phoenix6.signals.NeutralModeValue;
import com.revrobotics.spark.SparkLowLevel.MotorType;
import com.revrobotics.spark.SparkMax;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import org.littletonrobotics.junction.AutoLogOutput;

public class WristIOTalonFX implements WristIO {
  private TalonFX arm;
  private SparkMax rollers;
  private CANrange canRange;

  @AutoLogOutput private double angle;

  public WristIOTalonFX(int armID, int rollerID, String canbusName, int CanrangeID) {

    this.arm = new TalonFX(armID, canbusName);
    this.rollers = new SparkMax(rollerID, MotorType.kBrushless);
    this.canRange = new CANrange(CanrangeID);
  }

  @Override
  public void setArmSpeed(double speed) {
    arm.set(speed);
  }

  public void setRollerSpeed(double speed) {
    rollers.set(speed);
  }

  // @Override
  // public double getAngleRad() {
  //   angle = arm.getPosition().getValue().baseUnitMagnitude()*2*Math.PI;
  //   SmartDashboard.putNumber("Wrist angle: ", angle);
  //   return angle;
  // }

  @Override
  public double getAngleRotations() {
    angle = arm.getPosition().getValueAsDouble();
    SmartDashboard.putNumber("Wrist angle: ", angle);
    return angle;
  }

  @Override
  public void updateInputs(WristIOInputsAutoLogged inputs) {
    inputs.wristAppliedVoltage = arm.getMotorVoltage().getValueAsDouble();
    inputs.wristCurrentAmps = arm.getStatorCurrent().getValueAsDouble();
    inputs.wristSpeedRad = arm.get();
    inputs.wristLocationRotations = getAngleRotations();
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
    return canRange.getIsDetected().getValue();
  }

  @Override
  public double getDistance() {
    return canRange.getDistance().getValueAsDouble();
  }

  @Override
  public void zeroArmEncoder() {
    arm.setPosition(0);
  }
}
