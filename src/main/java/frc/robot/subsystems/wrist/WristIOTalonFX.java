package frc.robot.subsystems.wrist;

import com.ctre.phoenix6.configs.TalonFXConfiguration;
import com.ctre.phoenix6.controls.MotionMagicVoltage;
import com.ctre.phoenix6.hardware.CANrange;
import com.ctre.phoenix6.hardware.TalonFX;
import com.ctre.phoenix6.signals.NeutralModeValue;
import com.revrobotics.spark.SparkLowLevel.MotorType;
import com.revrobotics.spark.SparkMax;

import edu.wpi.first.units.Units;
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

     TalonFXConfiguration talonFXConfigs = new TalonFXConfiguration();


    var slot0Configs = talonFXConfigs.Slot0;
    slot0Configs.kS = 0.25; // Add 0.25 V output to overcome static friction
    slot0Configs.kV = 0.12; // A velocity target of 1 rps results in 0.12 V output
    slot0Configs.kA = 0.01; // An acceleration of 1 rps/s requires 0.01 V output
    slot0Configs.kP = 4.8; // A position error of 2.5 rotations results in 12 V output
    slot0Configs.kI = 0; // no output for integrated error
    slot0Configs.kD = 0.1; // A velocity error of 1 rps results in 0.1 V output

    var motionMagicConfigs = talonFXConfigs.MotionMagic;
    motionMagicConfigs.MotionMagicCruiseVelocity = 80; // Target cruise velocity of 80 rps
    motionMagicConfigs.MotionMagicAcceleration = 160; // Target acceleration of 160 rps/s (0.5 seconds)
    motionMagicConfigs.MotionMagicJerk = 1600; // Target jerk of 1600 rps/s/s (0.1 seconds)

    arm.getConfigurator().apply(talonFXConfigs);
  }

  @Override
  public void PIDVoltage(double targetAngle) {
    final MotionMagicVoltage m_request = new MotionMagicVoltage(0);

    // set target position to 100 rotations
    arm.setControl(m_request.withPosition(targetAngle));
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
    inputs.wristSpeedRotations = arm.get();
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
  public void setArmVoltage(double voltage) {
    arm.setVoltage(voltage);
  }

  @Override
  public void zeroArmEncoder() {
    arm.setPosition(0);
  }
}
