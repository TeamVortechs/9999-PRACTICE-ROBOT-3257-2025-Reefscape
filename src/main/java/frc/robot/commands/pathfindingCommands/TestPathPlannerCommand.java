package frc.robot.commands.pathfindingCommands;

import java.util.ArrayList;
import java.util.List;
import java.util.Random;

import com.pathplanner.lib.auto.AutoBuilder;
import com.pathplanner.lib.path.GoalEndState;
import com.pathplanner.lib.path.PathConstraints;
import com.pathplanner.lib.path.PathPlannerPath;
import com.pathplanner.lib.path.Waypoint;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.InstantCommand;
import frc.robot.subsystems.drive.Drive;

/*
Names
brief description
 */
public class TestPathPlannerCommand extends Command {
  @SuppressWarnings({"PMD.UnusedPrivateField", "PMD.SingularField"})

  /**
   * Creates a new ExampleCommand.
   *
   * @param subsystem The subsystem used by this command.
   */

    private Drive drive;
    private boolean ranCommand = false;
    private Command command = new InstantCommand();


  public TestPathPlannerCommand(Drive drive) {
    // addRequirements(null);
    this.drive = drive;
    
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {
    List<Waypoint> waypoints = PathPlannerPath.waypointsFromPoses(
        new Pose2d(5.782, 4.190, Rotation2d.fromDegrees(180)), new Pose2d(5.782, 4.190, Rotation2d.fromDegrees(180))
    );

    PathConstraints pathConstraints = new PathConstraints(3.0, 3.0, 2 * Math.PI, 4 * Math.PI);

    PathPlannerPath path = new PathPlannerPath(waypoints, pathConstraints, null, new GoalEndState(0.0, Rotation2d.fromDegrees(-180)));
    
    command = AutoBuilder.followPath(path);

    ranCommand = false;
  }

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {
    ranCommand = true;
    AutoBuilder.followPath()
  }

  // Called once the command ends or is interrupted.
  @Override
  public void end(boolean interrupted) {}

  // Returns true when the command should end.
  @Override
  public boolean isFinished() {
    return false;
  }
}
