package frc.robot.auton.pathfollowing;
import java.util.ArrayList;
import java.util.Arrays;
import java.util.Hashtable;

import frc.robot.auton.pathfollowing.PathBuilder.Waypoint;
import frc.robot.auton.pathfollowing.control.Path;
import frc.robot.auton.pathfollowing.motion.Translation;

import static frc.robot.auton.pathfollowing.PathBuilder.buildPathFromWaypoints;
import static frc.robot.auton.pathfollowing.PathBuilder.getStraightPathWaypoints;
import static frc.robot.auton.pathfollowing.PathBuilder.flipPath;
import static frc.robot.auton.pathfollowing.PathBuilder.reversePath;

public class Paths {
	private static Hashtable<Center, Path> centerPaths = new Hashtable<Center, Path>();
	private static Hashtable<Left, Path> leftPaths = new Hashtable<Left, Path>();
	private static Hashtable<Right, Path> rightPaths = new Hashtable<Right, Path>();
	
	public enum Center {
		AUTO_RUN,
		DEMO_PATH,
		DEMO_PATH_DOS,
		// First Cube
		L_SWITCH,
		R_SWITCH,
		
		// Second Cube
		L_SWITCH_TO_PYRAMID_FRONT,
		R_SWITCH_TO_PYRAMID_FRONT,
		
		TO_PYRAMID,
		FROM_PYRAMID,
		
		S_TURN_TO_L_SWITCH,
		S_TURN_TO_R_SWITCH,
		
		// Third Cube
		AWAY_FROM_L_SWITCH,
		PYRAMID_AGAIN_FROM_L,

		AWAY_FROM_R_SWITCH,
		PYRAMID_AGAIN_FROM_R,
	}
	private static Path demoPath;
	private static Path demoPathDos;
	private static Path autoRunPath;
	private static Path lSwitchPath, rSwitchPath;
	private static Path lSwitchToPyramidFrontPath, rSwitchToPyramidFrontPath;
	private static Path toPyramidPath, fromPyramidPath;
	private static Path sTurnToLSwitchPath, sTurnToRSwitchPath;
	private static Path awayFromLSwitchPath, awayFromRSwitchPath;
	private static Path pyramidAgainFromLeftPath, pyramidAgainFromRightPath;
	
	public enum Left 
	{
		TO_FRONT_CARGO_SHIP_L
	}
	
	private static Path _toFrontCargoShipLeft;
	
	public enum Right {
		TO_BACK_CENTER,
		TO_BACK_RIGHT,
		
		// First Cube
		L_SCALE,
		R_SCALE,
		
		R_SCALE_OUTSIDE,
		
		// Second Cube
		L_SCALE_TO_L_SWITCH,
		L_SWITCH_TO_L_SCALE,
		
		R_SCALE_TO_R_SWITCH,
		R_SWITCH_TO_R_SCALE,
		
		R_SWITCH_SIDE,
		
		// Third Cube
		R_SWITCH_TO_R_SCALE_SECOND_CUBE,
		
		R_SCALE_TO_R_SWITCH_THIRD_CUBE,
		R_SWITCH_TO_R_SCALE_THIRD_CUBE
	}

	
	
	public static void buildPaths() {
		buildCenterPaths();
		buildLeftPaths();
		buildRightPaths();
	}
	
	public static Path getPath(Center pathName) {
		return centerPaths.get(pathName);
	}
	
	public static Path getPath(Left pathName) {
		return leftPaths.get(pathName);
	}
	
	public static Path getPath(Right pathName) {
		return rightPaths.get(pathName);
	}
	
	private static void buildCenterPaths() 
	{
	
	}
	
	private static void buildLeftPaths() 
	{		
		_toFrontCargoShipLeft = buildPathFromWaypoints( Arrays.asList(
		new Waypoint(66,120,0,0),
		new Waypoint(115,120,20,20),
		new Waypoint(145,151,20,20),
		new Waypoint(185,151,0,20)));
		leftPaths.put(Left.TO_FRONT_CARGO_SHIP_L, _toFrontCargoShipLeft);
	}
	
	private static void buildRightPaths() 
	{
		
	}
}