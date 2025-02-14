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

import com.pathplanner.lib.auto.AutoBuilder;
import com.pathplanner.lib.path.PathConstraints;
import com.pathplanner.lib.path.PathPlannerPath;
import com.pathplanner.lib.path.Waypoint;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Transform2d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.util.Units;
import edu.wpi.first.wpilibj.DigitalInput;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import edu.wpi.first.wpilibj2.command.button.CommandXboxController;
import edu.wpi.first.wpilibj2.command.sysid.SysIdRoutine;
import frc.robot.KDoublePreferences.PElevator;
import frc.robot.commands.DriveCommands;
import frc.robot.commands.elevator.ElevatorHomeCommand;
import frc.robot.commands.elevator.SetElevatorPresetCommand;
import frc.robot.generated.TunerConstants;
import frc.robot.subsystems.drive.Drive;
import frc.robot.subsystems.drive.GyroIO;
import frc.robot.subsystems.drive.GyroIOPigeon2;
import frc.robot.subsystems.drive.ModuleIO;
import frc.robot.subsystems.drive.ModuleIOSim;
import frc.robot.subsystems.drive.ModuleIOTalonFX;
import frc.robot.subsystems.elevator.Elevator;
import frc.robot.subsystems.elevator.ElevatorModuleIO;
import frc.robot.subsystems.elevator.ElevatorModuleTalonFXIO;
import frc.robot.subsystems.vision.Vision;
import frc.robot.subsystems.vision.VisionConstants;
import frc.robot.subsystems.vision.VisionIO;
import frc.robot.subsystems.vision.VisionIOPhotonVisionSim;
import java.io.IOException;
import java.util.List;
import org.json.simple.parser.ParseException;
import org.littletonrobotics.junction.networktables.LoggedDashboardChooser;

/**
 * RobotContainer is the central hub for subsystems, commands, and operator interface bindings. It
 * instantiates all subsystems, sets up autonomous routines, and binds buttons to commands.
 */
public class RobotContainer {
  // Subsystems
  private final Drive drive;

  @SuppressWarnings("unused")
  private final Vision vision;

  // physical subsystems
  // private final Wrist wrist = new Wrist(new WristIOTalonFX());
  DigitalInput limitSwitch =
      new DigitalInput(20); // !!!!! FAKE CHANNEL! CHANGE WHEN PROPERLY IMPLEMENTED !!!!!!
  //   private final Intake intake = new Intake(new IntakeIOTalonFX(), limitSwitch);
  // private final Elevator elevator = new Elevator(eModuleIO);
  //   private final Elevator2 elevator2 =
  //       new Elevator2(
  //           new ElevatorModuleTalonFXIO(
  //               Constants.ELEVATOR_MOTOR_LEFT_ID,
  //               Constants.ELEVATOR_MOTOR_RIGHT_ID,
  //               Constants.ELEVATOR_CANBUS));

  // Intake subsystem with its limit switch (placeholder channel)
  //   @SuppressWarnings("unused")
  //   private final DigitalInput intakeLimitSwitch =
  //       new DigitalInput(20); // FAKE CHANNEL – update when implemented
  //   private final Intake intake = new Intake(new IntakeIOTalonFX(), intakeLimitSwitch);

  // Elevator subsystem: using our TalonFX-based IO and a dedicated home switch
  private final ElevatorModuleIO eModuleIO =
      new ElevatorModuleTalonFXIO(
          Constants.ELEVATOR_MOTOR_LEFT_ID,
          Constants.ELEVATOR_MOTOR_RIGHT_ID,
          Constants.ELEVATOR_CANBUS);
  private final DigitalInput elevatorHomeSwitch = new DigitalInput(20); // Update channel as needed
  private final Elevator elevator = new Elevator(eModuleIO, elevatorHomeSwitch);

  // Controllers for operator input
  private final CommandXboxController controller = new CommandXboxController(0);
  private final CommandXboxController controller2 = new CommandXboxController(1);

  // Autonomous chooser and path constraints
  private final LoggedDashboardChooser<Command> autoChooser;

  @SuppressWarnings("unused")
  private final PathConstraints pathConstraints =
      new PathConstraints(0.75, 0.5, Units.degreesToRadians(540), Units.degreesToRadians(720));

  /**
   * Constructs the RobotContainer by instantiating subsystems, configuring autonomous routines, and
   * binding controls.
   */
  public RobotContainer() {
    // Instantiate drive and vision subsystems based on the current mode.
    switch (Constants.currentMode) {
      case REAL:
        // Real robot: instantiate hardware IO implementations
        drive =
            new Drive(
                new GyroIOPigeon2(),
                new ModuleIOTalonFX(TunerConstants.FrontLeft),
                new ModuleIOTalonFX(TunerConstants.FrontRight),
                new ModuleIOTalonFX(TunerConstants.BackLeft),
                new ModuleIOTalonFX(TunerConstants.BackRight));
        vision =
            new Vision(
                drive::addVisionMeasurement, new VisionIO() {}
                // new VisionIOPhotonVision(
                //     VisionConstants.camera0Name, VisionConstants.robotToCamera0)
                // new VisionIOPhotonVision(camera1Name, robotToCamera1)
                );
        break;

      case SIM:
        // Simulation: instantiate sim IO implementations
        drive =
            new Drive(
                new GyroIO() {},
                new ModuleIOSim(TunerConstants.FrontLeft),
                new ModuleIOSim(TunerConstants.FrontRight),
                new ModuleIOSim(TunerConstants.BackLeft),
                new ModuleIOSim(TunerConstants.BackRight));
        vision =
            new Vision(
                drive::addVisionMeasurement,
                new VisionIOPhotonVisionSim(
                    VisionConstants.camera0Name, VisionConstants.robotToCamera0, drive::getPose));
        break;

      default:
        // Replayed robot: disable IO implementations
        drive =
            new Drive(
                new GyroIO() {},
                new ModuleIO() {},
                new ModuleIO() {},
                new ModuleIO() {},
                new ModuleIO() {});
        vision = new Vision(drive::addVisionMeasurement, new VisionIO() {});
        break;
    }

    // Set up autonomous routines using the PathPlanner auto builder.
    autoChooser = new LoggedDashboardChooser<>("Auto Choices", AutoBuilder.buildAutoChooser());
    autoChooser.addOption(
        "Drive Wheel Radius Characterization", DriveCommands.wheelRadiusCharacterization(drive));
    autoChooser.addOption(
        "Drive Simple FF Characterization", DriveCommands.feedforwardCharacterization(drive));
    autoChooser.addOption(
        "Drive SysId (Quasistatic Forward)",
        drive.sysIdQuasistatic(SysIdRoutine.Direction.kForward));
    autoChooser.addOption(
        "Drive SysId (Quasistatic Reverse)",
        drive.sysIdQuasistatic(SysIdRoutine.Direction.kReverse));
    autoChooser.addOption(
        "Drive SysId (Dynamic Forward)", drive.sysIdDynamic(SysIdRoutine.Direction.kForward));
    autoChooser.addOption(
        "Drive SysId (Dynamic Reverse)", drive.sysIdDynamic(SysIdRoutine.Direction.kReverse));

    // Configure button bindings.
    configureButtonBindings();
  }

  /** Configures operator interface button bindings. */
  private void configureButtonBindings() {
    // ----- Elevator Commands (using controller2) -----
    // Home the elevator when the B button is held.
    controller2.b().whileTrue(new ElevatorHomeCommand(elevator));

    // Move to preset heights using the bumper and trigger buttons.
    controller2
        .rightBumper()
        .whileTrue(new SetElevatorPresetCommand(elevator, PElevator.FirstLevel.getValue()));
    controller2
        .rightTrigger()
        .whileTrue(new SetElevatorPresetCommand(elevator, PElevator.SecondLevel.getValue()));
    controller2
        .leftTrigger()
        .whileTrue(new SetElevatorPresetCommand(elevator, PElevator.ThirdLevel.getValue()));

    // ----- Drive & Vision Commands (using controller) -----
    // Set the default command for the drive subsystem (field-relative joystick drive).
    // Default command, normal field-relative drive

    List<Waypoint> waypoints =
        PathPlannerPath.waypointsFromPoses(
            new Pose2d(1, 0, Rotation2d.fromDegrees(0)),
            new Pose2d(0, 1, Rotation2d.fromDegrees(1)));

    PathPlannerPath path = null;
    // new PathPlannerPath(
    //     waypoints,
    //     pathConstraints,
    //     null,
    //     new GoalEndState(0.0, Rotation2d.fromDegrees(0)),
    //     false);

    try {
      path = PathPlannerPath.fromPathFile("CoralFeed");
    } catch (IOException e) {
      System.out.println("IO exception");
    } catch (ParseException e) {
      System.out.println("parse exception ");
    }

    // controller.start().whileTrue(new WristSetPosCommand(wrist, 0.25));
    //  controller.back().whileTrue(new WristSetPosCommand(wrist, -0.25));
    // controller2.leftBumper().whileTrue(new IntakeSpeedCommand(intake, 0.75, limitSwitch));
    // controller.b().whileTrue(elevator2.runCurrentZeroing());
    // controller.rightBumper().whileTrue(new SetElevatorPower(elevator2, 0.1));
    // controller.leftBumper().whileTrue(new SetElevatorPower(elevator2, -0.1));
    /*  controller2
            .rightBumper()
            .whileTrue(new SetElevatorCommand(ElevatorLevel.FIRST_LEVEL, elevator));
        controller2
            .rightTrigger()
            .whileTrue(new SetElevatorCommand(ElevatorLevel.SECOND_LEVEL, elevator));
        controller2
            .leftTrigger()
            .whileTrue(new SetElevatorCommand(ElevatorLevel.THIRD_LEVEL, elevator));
        controller2
            .a()
            .whileTrue(
                new WristSetPosCommand(wrist, WristAngle.INTAKE_ANGLE)
                    .andThen(new SetWristRollerSpeed(wrist, -0.4)));
    /* */
    drive.setDefaultCommand(
        DriveCommands.joystickDrive(
            drive,
            () -> -controller.getLeftY(),
            () -> -controller.getLeftX(),
            () -> -controller.getRightX()));

    // Additional drive controls:
    // Lock drive to 0° when the A button is held.
    controller
        .a()
        .whileTrue(
            DriveCommands.joystickDriveAtAngle(
                drive,
                () -> -controller.getLeftY(),
                () -> -controller.getLeftX(),
                () -> new Rotation2d()));
    // Stop the drive with an X-pattern when the X button is pressed.
    controller.x().onTrue(Commands.runOnce(drive::stopWithX, drive));
    // Reset the gyro to 0° when the B button is pressed.
    controller
        .b()
        .onTrue(
            Commands.runOnce(
                    () ->
                        drive.setPose(
                            new Pose2d(drive.getPose().getTranslation(), new Rotation2d())),
                    drive)
                .ignoringDisable(true));
    // Apply a free disturbance to the drive pose (for vision testing) when the Y button is pressed.
    var disturbance =
        new Transform2d(new Translation2d(1.0, 1.0), new Rotation2d(0.17 * 2 * Math.PI));
    controller
        .y()
        .onTrue(
            Commands.runOnce(() -> drive.setPose(drive.getPose().plus(disturbance)))
                .ignoringDisable(true));
  }

  /**
   * Returns the autonomous command selected via the dashboard chooser.
   *
   * @return The autonomous command.
   */
  public Command getAutonomousCommand() {
    return autoChooser.get();
  }

  /** Puts drivetrain position data on the SmartDashboard. */
  public void putPositionData() {
    SmartDashboard.putNumber("x position:", drive.getPose().getX());
    SmartDashboard.putNumber("y position:", drive.getPose().getY());
    SmartDashboard.putNumber("current rotation:", drive.getPose().getRotation().getDegrees());
  }
}
