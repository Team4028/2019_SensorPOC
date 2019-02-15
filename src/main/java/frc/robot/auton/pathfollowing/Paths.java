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
		TO_FRONT_CARGO_SHIP_L,
		
		
	}
	private static Path _toFrontCargoShipLFromC;
	
	public enum Left 
	{
		TO_FRONT_CARGO_SHIP_L,
		TO_LEFT_CARGO_SHIP_FIRST,
		FROM_FIRST_BAY_TO_FEEDER_STATION,
		FROM_FRONT_CARGO_SHIP_L_TO_FEEDER_STATION;
	}
	
	private static Path _toFrontCargoShipLFromL;
	private static Path _toLeftCargoShipFirstBay;
	private static Path _toFeederStationFromFrontL;
	private static Path _toFeederStationFromFirstBayL;
	
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
		_toFrontCargoShipLFromC = buildPathFromWaypoints(Arrays.asList(
			new Waypoint(66,160,0,0),
			new Waypoint(115,160,20,20),
			new Waypoint(160,151,20,20),
			new Waypoint(185,151,0,20)
		));
		centerPaths.put(Center.TO_FRONT_CARGO_SHIP_L, _toFrontCargoShipLFromC);


		


	}
	
	private static void buildLeftPaths() 
	{		
		_toFrontCargoShipLFromL = buildPathFromWaypoints(Arrays.asList(
		new Waypoint(35,120,0,0),
		new Waypoint(135,120,20,20),
		new Waypoint(145,155,20,20),
		new Waypoint(185,155,0,20)));
		leftPaths.put(Left.TO_FRONT_CARGO_SHIP_L, _toFrontCargoShipLFromL);

		_toLeftCargoShipFirstBay = buildPathFromWaypoints(Arrays.asList(
			new Waypoint(66,120,0,0),
			new Waypoint(130,120,20,20),
			new Waypoint(160,80,20,20),
			new Waypoint(260,80,16,20),
			new Waypoint(260,100,0,20)
		));
		leftPaths.put(Left.TO_LEFT_CARGO_SHIP_FIRST, _toLeftCargoShipFirstBay);

		_toFeederStationFromFirstBayL = buildPathFromWaypoints(Arrays.asList(
			new Waypoint(260,115,0,0),
			new Waypoint(240,105,16,20),
			new Waypoint(150,95,30,20),
			new Waypoint(90,65,30,20),
			new Waypoint(20,65,0,20)
		));
		leftPaths.put(Left.FROM_FIRST_BAY_TO_FEEDER_STATION, _toFeederStationFromFirstBayL);

		_toFeederStationFromFrontL = buildPathFromWaypoints( Arrays.asList(
			new Waypoint(180,155,0,0),
			new Waypoint(130,35,35,20),
			new Waypoint(60,35,0,20)
		));
		leftPaths.put(Left.FROM_FRONT_CARGO_SHIP_L_TO_FEEDER_STATION, _toFeederStationFromFrontL);
	}
	
	private static void buildRightPaths() 
	{
		
	}
}