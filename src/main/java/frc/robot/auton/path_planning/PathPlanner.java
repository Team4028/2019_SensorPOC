package frc.robot.auton.path_planning;


import frc.robot.auton.path_planning.math.geometry;
import frc.robot.auton.path_planning.math.point;
import frc.robot.auton.pathfollowing.control.Path;
import frc.robot.auton.pathfollowing.motion.Translation;
import frc.robot.auton.pathfollowing.PathBuilder;
import frc.robot.auton.pathfollowing.PathBuilder.Waypoint;
import java.util.ArrayList;
import java.util.List;
import java.util.function.Function;
import java.lang.Math;

public class PathPlanner {

    public static Path planPath(List<Waypoint> w){
        return PathBuilder.buildPathFromWaypoints(w);                
    }
    
}