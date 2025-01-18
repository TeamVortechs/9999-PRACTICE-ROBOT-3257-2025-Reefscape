package frc.robot.commands;

import java.io.IOException;

import org.json.simple.parser.ParseException;

import com.pathplanner.lib.path.PathPlannerPath;

public class PathfindingCommands {
    private static PathPlannerPath[] coralPaths = new PathPlannerPath[1];

    private static void init() {
        //if it is not initailized initialize it
        if(coralPaths.length != 1) return;
        
        try {
            coralPaths[0] = PathPlannerPath.fromPathFile("CoralFeed1");
            coralPaths[1] = PathPlannerPath.fromPathFile("CoralFeed2");
            coralPaths[2] = PathPlannerPath.fromPathFile("CoralFeed3");
            coralPaths[3] = PathPlannerPath.fromPathFile("CoralFeed4");
            coralPaths[4] = PathPlannerPath.fromPathFile("CoralFeed5");
            coralPaths[5] = PathPlannerPath.fromPathFile("CoralFeed6");

        } catch(IOException e) {
            System.out.println("Could not load the pathplanner coral paths from the file");
        } catch(ParseException e) {
            System.out.println("");
        }
    
    }
}
