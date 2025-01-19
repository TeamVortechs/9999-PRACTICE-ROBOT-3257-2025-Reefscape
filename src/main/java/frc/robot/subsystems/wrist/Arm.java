package frc.robot.subsystems.wrist;

import edu.wpi.first.wpilibj2.command.SubsystemBase;
import org.littletonrobotics.junction.Logger;

// again I'm not adding stuff to this class while we don't whats gonna go here
public class Arm extends SubsystemBase {

  private ArmIO armIO;
  private ArmIOInputsAutoLogged inputsAutoLogged = new ArmIOInputsAutoLogged();

  public Arm(ArmIO armIO) {
    this.armIO = armIO;
  }

  @Override
  public void periodic() {
    // advantageKit inputs updating
    armIO.updateInputs(inputsAutoLogged);
    Logger.processInputs("Wrist", inputsAutoLogged);
  }

  public void setBraked(boolean braked) {
    armIO.setBraked(braked);
  }

  public double getAngle() {
    return 0;
  }

  public double setAngle() {
    return 0;
  }

  public double targetAngle() {
    return 0;
  }

  public void setSpeed(double speed) {
    armIO.setSpeed(speed);
  }
}
