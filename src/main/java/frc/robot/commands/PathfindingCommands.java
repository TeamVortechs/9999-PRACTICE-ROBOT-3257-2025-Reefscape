package frc.robot.commands;

import com.pathplanner.lib.auto.AutoBuilder;
import com.pathplanner.lib.path.PathConstraints;
import com.pathplanner.lib.path.PathPlannerPath;
import edu.wpi.first.math.util.Units;
import edu.wpi.first.wpilibj2.command.Command;
import java.io.IOException;
import org.json.simple.parser.ParseException;

public class PathfindingCommands {
  private static PathPlannerPath[] coralPaths = new PathPlannerPath[1];

  private static final PathConstraints pathConstraints =
      new PathConstraints(0.75, 0.5, Units.degreesToRadians(540), Units.degreesToRadians(720));

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
      System.out.println("Could not load the pathplanner coral paths from the file");
    } catch (ParseException e) {
      System.out.println("");
    }
  }

  public static Command pathfindToDepotCommand(int depotID) {
    init();
    return AutoBuilder.pathfindThenFollowPath(coralPaths[depotID], pathConstraints);
  }
}
