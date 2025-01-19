package frc.robot.commands;

import com.pathplanner.lib.auto.AutoBuilder;
import com.pathplanner.lib.path.PathConstraints;
import com.pathplanner.lib.path.PathPlannerPath;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.util.Units;
import edu.wpi.first.wpilibj2.command.Command;
import java.io.IOException;
import org.json.simple.parser.ParseException;

public class PathfindingCommands {
  private static PathPlannerPath[] coralPaths = new PathPlannerPath[1];

  private static final PathConstraints pathConstraints =
      new PathConstraints(0.75, 0.5, Units.degreesToRadians(540), Units.degreesToRadians(720));

  //creates the array of paths from hardcode path files
  private static void init() {
    // if it is not initailized initialize it
    if (coralPaths.length != 1) return;
    coralPaths = new PathPlannerPath[6];

    try {
      coralPaths[0] = PathPlannerPath.fromPathFile("CoralFeed1");
      coralPaths[1] = PathPlannerPath.fromPathFile("CoralFeed2");
      coralPaths[2] = PathPlannerPath.fromPathFile("CoralFeed3");
      coralPaths[3] = PathPlannerPath.fromPathFile("CoralFeed4");
      coralPaths[4] = PathPlannerPath.fromPathFile("CoralFeed5");
      coralPaths[5] = PathPlannerPath.fromPathFile("CoralFeed6");

    } catch (IOException e) {
      System.out.println("Could not load the pathplanner coral paths from the file, IO exception");
    } catch (ParseException e) {
      System.out.println("Could not load the pathplanner coral paths from the file, Parse exception");
    }
  }

  //returns the command that pathfinds the robot to the specific depot id
  public static Command pathfindToDepotCommand(int depotID) {
    //initializes the REPO if it isn't already
    init();
    return AutoBuilder.pathfindThenFollowPath(coralPaths[depotID], pathConstraints);
  }

  //does the same thing as before but with a little bit more functionality. Command before runs while the robot is getting in position and command in between runs while the robot is doing the path. command after runs after everything has been finished
  public static Command pathfindToDepotCommand(Command commandBefore, Command commandInBetween, Command commandLast, int depotID) {
    init();

    PathPlannerPath path = coralPaths[depotID];

    //gets the two needed commands: pathfinding to pose and path from pose
    Command pathfindCommand = AutoBuilder.pathfindToPose(path.getStartingDifferentialPose(), pathConstraints);
    Command pathCommand = AutoBuilder.followPath(path);

    //a bunch of command composition to due what the function says it does(ik know it's confusing)
    return commandBefore.alongWith(pathfindCommand).andThen(commandInBetween.alongWith(pathCommand)).andThen(commandLast);
  }

  //finds the closest path that goes to the depot
  public static int getClosestDepotPath(Pose2d curLocation) {

    double lowestDist = Double.MAX_VALUE;
    int lowestDistID = 0;

    for(int i = 0; i < coralPaths.length; i++) {
      Pose2d testPose = coralPaths[i].getStartingDifferentialPose();

      double dist = curLocation.getTranslation().getDistance(testPose.getTranslation());

      if(dist < lowestDist) {
        lowestDistID = i;
      }
    }

    return lowestDistID;
  }
}
