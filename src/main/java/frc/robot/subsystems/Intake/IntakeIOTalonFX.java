package frc.robot.subsystems.Intake;
import com.ctre.phoenix6.hardware.TalonFX;
import com.ctre.phoenix6.signals.NeutralModeValue;

import frc.robot.subsystems.wrist.WristIOInputsAutoLogged;
public class IntakeIOTalonFX implements IntakeIO{
    private TalonFX intakemotor = new TalonFX(0);

    public void SetMotorSpeed(double Speed){
        intakemotor.set(Speed);
    }

     public void updateInputs(IntakeIOInputsAutoLogged inputs){
        inputs.CurrentMotorSpeed = intakemotor.get();
        inputs.CurrentMotorVoltage = intakemotor.getMotorVoltage(true).getValueAsDouble();
        inputs.CurrentMotorAmps = intakemotor.getStatorCurrent().getValueAsDouble();
     }
}
