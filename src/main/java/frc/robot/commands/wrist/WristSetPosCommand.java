package frc.robot.commands.wrist;

import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.subsystems.wrist.Wrist;
import frc.robot.subsystems.wrist.Wrist.WristAngle;
import frc.robot.subsystems.wrist.WristIO;

/*
Names
brief description
 */
public class WristSetPosCommand extends Command {
  @SuppressWarnings({"PMD.UnusedPrivateField", "PMD.SingularField"})
  private final Wrist wrist;

  private double targetAngle;

  private static WristAngle wristangle;



  /**
   * Creates a new ExampleCommand.
   *
   * @param subsystem The subsystem used by this command.
   */
  public WristSetPosCommand(Wrist wrist, double angle) {
    // addRequirements(null);
    this.wrist = wrist;
    this.targetAngle= angle;
  }

  public WristSetPosCommand(Wrist wrist, WristAngle wristangle){
    this.wrist = wrist;
    this.targetAngle = wristangle.getAngle();
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {
    wrist.setTargetAngle(targetAngle);
  }

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {
    // if(wrist.isOnTarget()){
    //   if(angle == wrist.intakeAngle){
    //     wrist.setRollerSpeed(-0.4);
    //   }
    //   else{
    //     wrist.setRollerSpeed(0.4);
    //   }
    // }
  }

  // Called once the command ends or is interrupted.
  @Override
  public void end(boolean interrupted) {
    wrist.setRollerSpeed(0);
  }

  // Returns true when the command should end.
  @Override
  public boolean isFinished() {
    if(wrist.isOnTarget()) {
      return true;
    }

    if(Math.abs(targetAngle - wrist.getTargetAngle()) > 0.001) {
      return true;
    }

    return false;
  }
}
