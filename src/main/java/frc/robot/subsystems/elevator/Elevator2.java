package frc.robot.subsystems.elevator;

import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import org.littletonrobotics.junction.Logger;

public class Elevator2 extends SubsystemBase {

  private ElevatorModuleIO elevatorModuleIO;

  private ElevatorModuleIOInputsAutoLogged inputs = new ElevatorModuleIOInputsAutoLogged();

  public Elevator2(ElevatorModuleIO elevatorModuleIO) {
    this.elevatorModuleIO = elevatorModuleIO;
  }

  public Command runCurrentZeroing() {
    System.out.println("Elevator is Homed");
    return this.run(() -> elevatorModuleIO.setVoltage(-0.05))
        .until(() -> (inputs.elevatorMotor1CurrentAmps > 40))
        .finallyDo(() -> elevatorModuleIO.resetEncoder());
  }

  @Override
  public void periodic() {
    elevatorModuleIO.updateInputs(inputs);
    Logger.processInputs("Elevator2", inputs);
  }

  public void setSpeed(double speed) {
    elevatorModuleIO.setSpeed(speed);
  }
}
