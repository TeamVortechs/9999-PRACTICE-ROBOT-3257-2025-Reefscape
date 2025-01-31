package frc.robot.subsystems.elevator;

import com.ctre.phoenix6.hardware.TalonFX;

public class ElevatorModuleSparkMaxIO implements ElevatorModuleIO {
    private TalonFX Elevatormotor = new TalonFX(0);
    private TalonFX Elevatormotor1 = new TalonFX(0);

    public void setSpeed(double speed){
        Elevatormotor.set(speed);
        Elevatormotor1.set(speed);
    }
    
}
