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
import com.pathplanner.lib.auto.NamedCommands;
import com.pathplanner.lib.path.PathConstraints;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.util.Units;
import edu.wpi.first.wpilibj.GenericHID;
import edu.wpi.first.wpilibj.XboxController;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import edu.wpi.first.wpilibj2.command.InstantCommand;
import edu.wpi.first.wpilibj2.command.button.CommandXboxController;
import edu.wpi.first.wpilibj2.command.sysid.SysIdRoutine;
import frc.robot.KDoublePreferences.PElevator;
import frc.robot.commands.autoCommands.DriveCommands;
import frc.robot.commands.communication.ControllerVibrateCommand;
import frc.robot.commands.communication.TellCommand;
import frc.robot.commands.wrist.IntakeWristCommand;
// import frc.robot.commands.SetWristRollerSpeed;
import frc.robot.commands.wrist.ManualSetWristSpeedCommand;
import frc.robot.commands.wrist.SetWristRollerSpeedCommand;
import frc.robot.commands.wrist.SetWristTargetAngleCommand;
import frc.robot.generated.TunerConstants;
// import frc.robot.subsystems.Intake.Intake;
// import frc.robot.subsystems.Intake.IntakeIOTalonFX;
import frc.robot.subsystems.drive.Drive;
import frc.robot.subsystems.drive.GyroIO;
import frc.robot.subsystems.drive.GyroIOPigeon2;
import frc.robot.subsystems.drive.ModuleIO;
import frc.robot.subsystems.drive.ModuleIOSim;
import frc.robot.subsystems.drive.ModuleIOTalonFX;
import frc.robot.subsystems.elevator.Elevator;
// import frc.robot.subsystems.elevator.Elevator2;
import frc.robot.subsystems.elevator.ElevatorModuleTalonFXIO;
import frc.robot.subsystems.vision.Vision;
import frc.robot.subsystems.vision.VisionConstants;
import frc.robot.subsystems.vision.VisionIO;
import frc.robot.subsystems.vision.VisionIOPhotonVisionSim;
import frc.robot.subsystems.wrist.Wrist;
import frc.robot.subsystems.wrist.Wrist.WristAngle;
// import frc.robot.subsystems.wrist.WristIOTalonFX;
import frc.robot.subsystems.wrist.WristIOTalonFX;
import org.littletonrobotics.junction.networktables.LoggedDashboardChooser;

/**
 * This class is where the bulk of the robot should be declared. Since Command-based is a
 * "declarative" paradigm, very little robot logic should actually be handled in the {@link Robot}
 * periodic methods (other than the scheduler calls). Instead, the structure of the robot (including
 * subsystems, commands, and button mappings) should be declared here.
 */
public class RobotContainer {
  // Subsystems
  private final Drive drive;

  @SuppressWarnings("unused")
  private final Vision vision;

  // physical subsystems
  private final Wrist wrist =
      new Wrist(
          new WristIOTalonFX(
              Constants.ARM_MOTOR_ID,
              Constants.ROLLER_MOTOR_ID,
              Constants.ELEVATOR_CANBUS,
              Constants.CANRANGE_ID));

  // DigitalInput limitSwitch =
  // new DigitalInput(20); // !!!!! FAKE CHANNEL! CHANGE WHEN PROPERLY IMPLEMENTED !!!!!!
  // private final Intake intake = new Intake(new IntakeIOTalonFX(), limitSwitch);
  private final Elevator elevator =
      new Elevator(
          new ElevatorModuleTalonFXIO(
              Constants.ELEVATOR_MOTOR_LEFT_ID,
              Constants.ELEVATOR_MOTOR_RIGHT_ID,
              Constants.ELEVATOR_CANBUS),
          wrist);
  //   private final Elevator2 elevator2 =
  //       new Elevator2(
  //           new ElevatorModuleTalonFXIO(
  //               Constants.ELEVATOR_MOTOR_LEFT_ID,
  //               Constants.ELEVATOR_MOTOR_RIGHT_ID,
  //               Constants.ELEVATOR_CANBUS));

  // Controller
  private final CommandXboxController controller = new CommandXboxController(0);
  private final CommandXboxController controller2 = new CommandXboxController(1);

  // Dashboard inputs
  private final LoggedDashboardChooser<Command> autoChooser;

  private final PathConstraints pathConstraints =
      new PathConstraints(0.75, 0.5, Units.degreesToRadians(540), Units.degreesToRadians(720));

  /** The container for the robot. Contains subsystems, OI devices, and commands. */
  public RobotContainer() {
    switch (Constants.currentMode) {
      case REAL:
        // Real robot, instantiate hardware IO implementations
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
        // Sim robot, instantiate physics sim IO implementations
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
        // Replayed robot, disable IO implementations
        drive =
            new Drive(
                new GyroIO() {},
                new ModuleIO() {},
                new ModuleIO() {},
                new ModuleIO() {},
                new ModuleIO() {});
        vision =
            new Vision(
                drive::addVisionMeasurement, new VisionIO() {}
                // new VisionIO() {}
                );
        break;
    }

    // Set up auto routines
    autoChooser = new LoggedDashboardChooser<>("Auto Choices", AutoBuilder.buildAutoChooser());

    // Set up SysId routines
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

    // configure the autonomous named commands
    registerNamedCommandsAuto();

    // Configure the button bindings
    configureButtonBindings();
  }

  /**
   * Use this method to define your button->command mappings. Buttons can be created by
   * instantiating a {@link GenericHID} or one of its subclasses ({@link
   * edu.wpi.first.wpilibj.Joystick} or {@link XboxController}), and then passing it to a {@link
   * edu.wpi.first.wpilibj2.command.button.JoystickButton}.
   */
  private void configureButtonBindings() {
    // Default command, normal field-relative drive

    // controller.start().whileTrue(new WristSetPosCommand(wrist, 0.25));
    //  controller.back().whileTrue(new WristSetPosCommand(wrist, -0.25));
    // controller2.leftBumper().whileTrue(new IntakeSpeedCommand(intake, 0.75, limitSwitch));
    // controller.b().whileTrue(elevator2.runCurrentZeroing());
    // controller.rightBumper().whileTrue(new SetElevatorPower(elevator2, 0.1));
    // controller.leftBumper().whileTrue(new SetElevatorPower(elevator2, -0.1));
    // controller
    //     .rightBumper()
    //     .whileTrue(new SetElevatorCommand(ElevatorLevel.FIRST_LEVEL, elevator2));
    // controller
    //     .rightTrigger()
    //     .whileTrue(new SetElevatorCommand(ElevatorLevel.SECOND_LEVEL, elevator2));
    // controller
    //     .leftTrigger()
    //     .whileTrue(new SetElevatorCommand(ElevatorLevel.THIRD_LEVEL, elevator2));

    // controller
    //     .a()
    //     .whileTrue(
    //         new TellCommand()
    //             .andThen(
    //                 new SetWristRollerSpeed(wrist, -0.01)
    //                     .unless(() -> wrist.isCanCloserThan(0.1))));

    wrist.setDefaultCommand(
        new SetWristTargetAngleCommand(wrist, WristAngle.STAGE1_ANGLE.getAngle())
            .unless(() -> !wrist.isCanCloserThan(0.1)));

    controller
        .rightBumper()
        .onTrue(
            new InstantCommand(() -> elevator.resetEncoders())
                .ignoringDisable(true)
                .alongWith(new InstantCommand(() -> wrist.resetWristEncoder()))
                .ignoringDisable(true));

    controller.b().whileTrue(new ManualSetWristSpeedCommand(wrist, () -> 0.05));
    // controller.x().whileTrue(new SetWristTargetAngleCommand(wrist, 0));
    controller
        .a()
        .whileTrue(
            new IntakeWristCommand(wrist, -0.2)
                .andThen(new ControllerVibrateCommand(0.7, controller)));

    controller.y().whileTrue(new SetWristRollerSpeedCommand(wrist, -0.5));
    controller
        .leftBumper()
        .whileTrue(new SetWristTargetAngleCommand(wrist, WristAngle.STAGE1_ANGLE.getAngle()));

    // controller.leftTrigger().whileTrue(new ManualElevatorCommand(elevator, () -> -0.2));
    // controller.rightTrigger().whileTrue(new ManualElevatorCommand(elevator, () -> 0.2));
    controller
        .leftTrigger()
        .whileTrue(
            new InstantCommand(() -> elevator.setTargetHeight(PElevator.FirstLevel.getValue())));
    controller
        .rightTrigger()
        .whileTrue(
            new InstantCommand(() -> elevator.setTargetHeight(PElevator.SecondLevel.getValue())));
    controller
        .x()
        .whileTrue(
            new InstantCommand(() -> elevator.setTargetHeight(PElevator.MinHeight.getValue())));
    drive.setDefaultCommand(
        DriveCommands.joystickDrive(
            drive,
            () -> -controller.getLeftY(),
            () -> -controller.getLeftX(),
            () -> -controller.getRightX()));

    // Lock to 0° when A button is held
    // controller
    //     .a()
    //     .whileTrue(
    //         DriveCommands.joystickDriveAtAngle(
    //             drive,
    //             () -> -controller.getLeftY(),
    //             () -> -controller.getLeftX(),
    //             () -> new Rotation2d( )));

    // Switch to X pattern when X button is pressed
    // controller.x().onTrue(Commands.runOnce(drive::stopWithX, drive));

    // Reset gyro to 0° when B button is pressed
    controller
        .povDown()
        .onTrue(
            Commands.runOnce(
                    () ->
                        drive.setPose(
                            new Pose2d(drive.getPose().getTranslation(), new Rotation2d())),
                    drive)
                .ignoringDisable(true));

    controller
        .povUp()
        .onTrue(
            Commands.runOnce(
                    () ->
                        drive.setPose(
                            new Pose2d(
                                drive.getPose().getTranslation(), Rotation2d.fromDegrees(180))),
                    drive)
                .ignoringDisable(true));

    // add a free disturbance when pressing the y button to test vision
    // var disturbance =
    //     new Transform2d(new Translation2d(1.0, 1.0), new Rotation2d(0.17 * 2 * Math.PI));
    // controller
    //     .y()
    //     .onTrue(
    //         Commands.runOnce(() -> drive.setPose(drive.getPose().plus(disturbance)))
    //             .ignoringDisable(true));
  } // end configure bindings

  /**
   * Use this to pass the autonomous command to the main {@link Robot} class.
   *
   * @return the command to run in autonomous
   */
  public Command getAutonomousCommand() {
    return autoChooser.get();
  }

  // registers pathplanner's named commands
  private void registerNamedCommandsAuto() {
    NamedCommands.registerCommand("testing", new TellCommand("testing autoCOmmand"));
  }

  //   public void sendVisionMeasurement() {
  //     // Correct pose estimate with vision measurements
  //     var visionEst = vision.getEstimatedGlobalPose();
  //     visionEst.ifPresent(
  //         est -> {
  //           // Change our trust in the measurement based on the tags we can see
  //           var estStdDevs = vision.getEstimationStdDevs();

  //           drive.addVisionMeasurement(
  //               est.estimatedPose.toPose2d(),
  //               est.timestampSeconds,
  //               estStdDevs); // !!! note: the standard deviation in the constants has to be
  // tweaked
  //         });
  //   }

  // intended for testing usage only
  // puts sendables on shuffleboard
  public void putPositionData() {
    SmartDashboard.putNumber("x position:", drive.getPose().getX());
    SmartDashboard.putNumber("y position:", drive.getPose().getY());
    SmartDashboard.putNumber("current rotation:", drive.getPose().getRotation().getDegrees());
  }
}
