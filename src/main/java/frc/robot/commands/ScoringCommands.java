package frc.robot.commands;

import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.WaitCommand;
import edu.wpi.first.wpilibj2.command.WaitUntilCommand;
import frc.robot.subsystems.elevator.Elevator;
import frc.robot.subsystems.wrist.Wrist;

public class ScoringCommands {
    //WPI command the moves the arm into position for scoring but dosn't actually score yet
    public static Command prepScoreCommand(int level, Elevator elevator, Wrist wrist) {
        return new WaitCommand(1);
    }

    //moves the arm into position and then scores
    public static Command scoreCommand(int level, Elevator elevator, Wrist wrist) {
        return new WaitCommand(0.01);
    }

    public static Command prepForStationIntakeCommand(Elevator elevator, Wrist wrist) {
        return new WaitCommand(1);
    }
}
