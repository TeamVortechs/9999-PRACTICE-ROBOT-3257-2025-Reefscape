package frc.robot.subsystems.Intake;

import com.ctre.phoenix6.hardware.TalonFX;

public class IntakeIOTalonFX implements IntakeIO {
  private TalonFX intakemotor;

  public IntakeIOTalonFX(int intakeMotorID) {
    this.intakemotor = new TalonFX(intakeMotorID);
  }

  public void SetMotorSpeed(double Speed) {
    intakemotor.set(Speed);
  }

  public void updateInputs(IntakeIOInputsAutoLogged inputs) {
    inputs.CurrentMotorSpeed = intakemotor.get();
    inputs.CurrentMotorVoltage = intakemotor.getMotorVoltage(true).getValueAsDouble();
    inputs.CurrentMotorAmps = intakemotor.getStatorCurrent().getValueAsDouble();
  }
}
