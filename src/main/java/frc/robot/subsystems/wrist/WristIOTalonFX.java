package frc.robot.subsystems.wrist;

import com.ctre.phoenix6.configs.TalonFXConfiguration;
import com.ctre.phoenix6.controls.MotionMagicVoltage;
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

    TalonFXConfiguration talonFXConfigs = new TalonFXConfiguration();

    var slot0Configs = talonFXConfigs.Slot0;
    slot0Configs.kS = 0.25; // Add 0.25 V output to overcome static friction
    slot0Configs.kV = 0.12; // A velocity target of 1 rps results in 0.12 V output
    slot0Configs.kA = 0.01; // An acceleration of 1 rps/s requires 0.01 V output
    slot0Configs.kP = 4.8; // A position error of 2.5 rotations results in 12 V output
    slot0Configs.kI = 0; // no output for integrated error
    slot0Configs.kD = 0.1; // A velocity error of 1 rps results in 0.1 V output

    var motionMagicConfigs = talonFXConfigs.MotionMagic;
    motionMagicConfigs.MotionMagicCruiseVelocity = 3; // Target cruise velocity of 80 rps
    motionMagicConfigs.MotionMagicAcceleration =
        2.5; // Target acceleration of 160 rps/s (0.5 seconds)
    motionMagicConfigs.MotionMagicJerk = 10; // Target jerk of 1600 rps/s/s (0.1 seconds)

    arm.getConfigurator().apply(talonFXConfigs);

    arm.setNeutralMode(NeutralModeValue.Brake);
  }

  // sets the PID target angle
  @Override
  public void PIDVoltage(double targetAngle) {
    final MotionMagicVoltage m_request = new MotionMagicVoltage(0);

    // set target position to 100 rotations
    arm.setControl(m_request.withPosition(targetAngle));
    System.out.println("Voltage being sent in PID Voltage");
  }

  // sets the speed of the arm
  @Override
  public void setArmSpeed(double speed) {
    arm.set(speed);
  }

  // sets the speed of the orllers
  public void setRollerSpeed(double speed) {
    rollers.set(speed);
  }

  // gets the angle of the arm
  @Override
  public double getAngleRotations() {
    angle = arm.getPosition().getValueAsDouble();
    SmartDashboard.putNumber("Wrist angle: ", angle);
    return angle;
  }

  // advantage kti logging stuff
  @Override
  public void updateInputs(WristIOInputsAutoLogged inputs) {
    inputs.wristAppliedVoltage = arm.getMotorVoltage().getValueAsDouble();
    inputs.wristCurrentAmps = arm.getStatorCurrent().getValueAsDouble();
    inputs.wristSpeedRotations = arm.get();
    inputs.wristLocationRotations = getAngleRotations();

    inputs.canRangeDistance = canRange.getDistance().getValueAsDouble();

    inputs.rollersCurrent = rollers.getOutputCurrent();
    inputs.rollersEncoder = rollers.getEncoder().getPosition();
    inputs.rollersSpeed = rollers.get();
  }

  // sets the arm motor to brake(DOES ACTUALY STOP THE ARM MOTORO)
  @Override
  public void setBraked(boolean braked) {
    if (braked) {
      arm.setNeutralMode(NeutralModeValue.Brake);
    } else {
      arm.setNeutralMode(NeutralModeValue.Coast);
    }
  }

  // returns wether or not the can range is detected
  @Override
  public boolean isDetected() {
    return canRange.getIsDetected().getValue();
  }

  // gets the distance from the can range
  @Override
  public double getDistance() {
    return canRange.getDistance().getValueAsDouble();
  }

  // sets the voltage of the arm
  @Override
  public void setArmVoltage(double voltage) {
    arm.setVoltage(voltage);
  }

  // resets the arm encoder
  @Override
  public void zeroArmEncoder() {
    arm.setPosition(0);
  }
}
