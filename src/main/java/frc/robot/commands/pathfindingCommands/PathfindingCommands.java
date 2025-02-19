package frc.robot.commands.pathfindingCommands;

import com.pathplanner.lib.auto.AutoBuilder;
import com.pathplanner.lib.path.PathConstraints;
import com.pathplanner.lib.path.PathPlannerPath;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.util.Units;
import edu.wpi.first.wpilibj2.command.Command;
import java.io.IOException;
import org.json.simple.parser.ParseException;

public class PathfindingCommands {
  private static PathPlannerPath[] coralPathsLeft = new PathPlannerPath[1];
  private static PathPlannerPath[] coralPathsRight = new PathPlannerPath[1];

  private static boolean initialized = false;

  private static final PathConstraints pathConstraints =
      new PathConstraints(0.75, 0.5, Units.degreesToRadians(540), Units.degreesToRadians(720));

  // creates the array of paths from hardcode path files
  private static void init() {
    // if it is not initailized initialize it
    if (initialized) return;

    initialized = true;
    coralPathsLeft = new PathPlannerPath[6];
    coralPathsRight = new PathPlannerPath[6];

    // loads all the different paths from their file names
    try {
      coralPathsLeft[0] = PathPlannerPath.fromPathFile("CoralFeed1 Left");
      coralPathsLeft[1] = PathPlannerPath.fromPathFile("CoralFeed2 Left");
      coralPathsLeft[2] = PathPlannerPath.fromPathFile("CoralFeed3 Left");
      coralPathsLeft[3] = PathPlannerPath.fromPathFile("CoralFeed4 Left");
      coralPathsLeft[4] = PathPlannerPath.fromPathFile("CoralFeed5 Left");
      coralPathsLeft[5] = PathPlannerPath.fromPathFile("CoralFeed6 Left");

      coralPathsRight[0] = PathPlannerPath.fromPathFile("CoralFeed1 Right");
      coralPathsRight[1] = PathPlannerPath.fromPathFile("CoralFeed2 Right");
      coralPathsRight[2] = PathPlannerPath.fromPathFile("CoralFeed3 Right");
      coralPathsRight[3] = PathPlannerPath.fromPathFile("CoralFeed4 Right");
      coralPathsRight[4] = PathPlannerPath.fromPathFile("CoralFeed5 Right");
      coralPathsRight[5] = PathPlannerPath.fromPathFile("CoralFeed6 Right");

      // for (int i = 0; i < coralPathsLeft.length; i++) {
      //   System.out.println(coralPathsLeft[i].name);
      //   System.out.println(coralPathsRight[i].name);
      // }

      // avoids exceptions
    } catch (IOException e) {
      System.out.println("Could not load the pathplanner coral paths from the file, IO exception");
    } catch (ParseException e) {
      System.out.println(
          "Could not load the pathplanner coral paths from the file, Parse exception");
    }
  }

  // DEPOT COMMANDS

  // returns the command that pathfinds the robot to the specific depot id
  public static Command pathfindToDepotCommand(int depotID, boolean left) {
    // initializes the REPO if it isn't already
    init();
    // System.out.println("depot id: " + depotID);
    // System.out.println("path with depot: " +
    // coralPaths[depotID].getPathPoses().get(0).toString());

    // for (int i = 0; i < coralPaths.length; i++) {
    //   System.out.println(i + " " + coralPaths[i].getPathPoses().get(0));
    // }
    if (left) {
      return AutoBuilder.pathfindThenFollowPath(coralPathsLeft[depotID], pathConstraints);
    } else {
      return AutoBuilder.pathfindThenFollowPath(coralPathsRight[depotID], pathConstraints);
    }
  }

  // finds the closest path that goes to the depot
  public static int getClosestDepotPath(Pose2d curLocation, boolean left) {

    init();

    double lowestDist = Double.MAX_VALUE;
    int lowestDistID = 0;

    // System.out.println("pose " + curLocation);
    if (left) {
      for (int i = 0; i < coralPathsLeft.length; i++) {
        Pose2d testPose = coralPathsLeft[i].getPathPoses().get(0);

        double dist = testPose.getTranslation().getDistance(curLocation.getTranslation());

        if (dist < lowestDist) {
          lowestDistID = i;
          lowestDist = dist;
        }

        // System.out.println("pose: " + testPose.toString() + " dist " + dist + " id: " +
        // lowestDistID);
      }
    } else {
      for (int i = 0; i < coralPathsRight.length; i++) {
        Pose2d testPose = coralPathsRight[i].getPathPoses().get(0);

        double dist = testPose.getTranslation().getDistance(curLocation.getTranslation());

        if (dist < lowestDist) {
          lowestDistID = i;
          lowestDist = dist;
        }

        // System.out.println("pose: " + testPose.toString() + " dist " + dist + " id: " +
        // lowestDistID);
      }
    }

    // System.out.println("lowest dist ID: " + lowestDistID);

    return lowestDistID;
  }
}
