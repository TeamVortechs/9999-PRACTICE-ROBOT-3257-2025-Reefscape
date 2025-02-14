// Copyright 2021-2025 FRC 6328
// http://github.com/Mechanical-Advantage
//
// This program is free software; you can redistribute it and/or
// modify it under the terms of the GNU General Public License
// version 3 as published by the Free Software Foundation or
// available in the root directory of this project.
//
// This program is distributed in the hope that it will be useful,
// but WITHOUT ANY WARRANTY; without even the implied warranty of
// MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE. See the
// GNU General Public License for more details.

package frc.robot;

import com.ctre.phoenix6.swerve.SwerveModuleConstants;
import com.ctre.phoenix6.swerve.SwerveModuleConstants.DriveMotorArrangement;
import com.ctre.phoenix6.swerve.SwerveModuleConstants.SteerMotorArrangement;
import com.pathplanner.lib.commands.PathfindingCommand;
import com.pathplanner.lib.pathfinding.Pathfinding;
import edu.wpi.first.wpilibj.Threads;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.CommandScheduler;
import frc.robot.generated.TunerConstants;
import frc.robot.util.FieldMovement.LocalADStarAK;
import org.littletonrobotics.junction.LogFileUtil;
import org.littletonrobotics.junction.LoggedRobot;
import org.littletonrobotics.junction.Logger;
import org.littletonrobotics.junction.networktables.NT4Publisher;
import org.littletonrobotics.junction.wpilog.WPILOGReader;
import org.littletonrobotics.junction.wpilog.WPILOGWriter;

/**
 * The Robot class is the main entry point for the robot code. It instantiates the
 * RobotContainer—which now includes our new Elevator subsystem— sets up logging, and runs the
 * command scheduler.
 */
public class Robot extends LoggedRobot {
  private Command autonomousCommand;
  private RobotContainer robotContainer;

  public Robot() {
    // Record build metadata
    Logger.recordMetadata("ProjectName", BuildConstants.MAVEN_NAME);
    Logger.recordMetadata("BuildDate", BuildConstants.BUILD_DATE);
    Logger.recordMetadata("GitSHA", BuildConstants.GIT_SHA);
    Logger.recordMetadata("GitDate", BuildConstants.GIT_DATE);
    Logger.recordMetadata("GitBranch", BuildConstants.GIT_BRANCH);
    switch (BuildConstants.DIRTY) {
      case 0:
        Logger.recordMetadata("GitDirty", "All changes committed");
        break;
      case 1:
        Logger.recordMetadata("GitDirty", "Uncomitted changes");
        break;
      default:
        Logger.recordMetadata("GitDirty", "Unknown");
        break;
    }

    // Set up data receivers and replay source based on the current mode.
    switch (Constants.currentMode) {
      case REAL:
        // For a real robot, log to a USB stick ("/U/logs")
        Logger.addDataReceiver(new WPILOGWriter());
        Logger.addDataReceiver(new NT4Publisher());
        break;
      case SIM:
        // For simulation, log to NT
        Logger.addDataReceiver(new NT4Publisher());
        break;
      case REPLAY:
        // For log replay, disable timing and set up file-based logging.
        setUseTiming(false);
        String logPath = LogFileUtil.findReplayLog();
        Logger.setReplaySource(new WPILOGReader(logPath));
        Logger.addDataReceiver(new WPILOGWriter(LogFileUtil.addPathSuffix(logPath, "_sim")));
        break;
    }

    // Start AdvantageKit logger.
    Logger.start();

    // Validate swerve configuration.
    SwerveModuleConstants[] modules = {
      TunerConstants.FrontLeft,
      TunerConstants.FrontRight,
      TunerConstants.BackLeft,
      TunerConstants.BackRight
    };
    for (SwerveModuleConstants constants : modules) {
      if (constants.DriveMotorType != DriveMotorArrangement.TalonFX_Integrated
          || constants.SteerMotorType != SteerMotorArrangement.TalonFX_Integrated) {
        throw new RuntimeException(
            "Unsupported swerve configuration. Please check your motor configurations.");
      }
    }

    // Instantiate RobotContainer.
    // Note: RobotContainer now instantiates and manages all subsystems—including the new Elevator.
    robotContainer = new RobotContainer();
  }

  @Override
  public void robotInit() {
    // Initialize pathfinding and warm up any auto commands.
    Pathfinding.setPathfinder(new LocalADStarAK());
    PathfindingCommand.warmupCommand();
  }

  @Override
  public void robotPeriodic() {
    // Increase thread priority to improve command scheduling.
    Threads.setCurrentThreadPriority(true, 99);
    CommandScheduler.getInstance().run();
    Threads.setCurrentThreadPriority(false, 10);
    // Update dashboard position data (includes drive pose; elevator data is handled in its
    // subsystem)
    robotContainer.putPositionData();
  }

  @Override
  public void disabledInit() {}

  @Override
  public void disabledPeriodic() {}

  @Override
  public void autonomousInit() {
    autonomousCommand = robotContainer.getAutonomousCommand();
    if (autonomousCommand != null) {
      autonomousCommand.schedule();
    }
  }

  @Override
  public void autonomousPeriodic() {}

  @Override
  public void teleopInit() {
    // Cancel the autonomous command when teleop begins.
    if (autonomousCommand != null) {
      autonomousCommand.cancel();
    }
  }

  @Override
  public void teleopPeriodic() {}

  @Override
  public void testInit() {
    // Cancel all running commands when test mode starts.
    CommandScheduler.getInstance().cancelAll();
  }

  @Override
  public void testPeriodic() {}

  @Override
  public void simulationInit() {}

  @Override
  public void simulationPeriodic() {}
}
