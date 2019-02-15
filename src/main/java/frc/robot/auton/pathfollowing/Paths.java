package frc.robot.auton.pathfollowing;
import java.util.Arrays;
import java.util.Hashtable;

import frc.robot.auton.pathfollowing.PathBuilder.Waypoint;
import frc.robot.auton.pathfollowing.control.Path;

import static frc.robot.auton.pathfollowing.PathBuilder.buildPathFromWaypoints;

public class Paths {
	private static Hashtable<Center, Path> centerPaths = new Hashtable<Center, Path>();
	private static Hashtable<Left, Path> leftPaths = new Hashtable<Left, Path>();
	private static Hashtable<Right, Path> rightPaths = new Hashtable<Right, Path>();
	
	public enum Center {
		TO_FRONT_CARGO_SHIP_L,
	}

	private static Path _toFrontCargoShipLFromC;
	
	public enum Left {
		TO_FRONT_CARGO_SHIP_L,
		TO_LEFT_CARGO_SHIP_FIRST,
		FROM_FIRST_BAY_TO_FEEDER_STATION,
		FROM_FRONT_CARGO_SHIP_L_TO_FEEDER_STATION,
		FROM_FEEDER_STATION_TO_FIRST_BAY,
		FROM_FEEDER_STATION_TO_SECOND_BAY;
	}
	
	private static Path _toFrontCargoShipLFromL;
	private static Path _toLeftCargoShipFirstBay;
	private static Path _toFeederStationFromFrontL;
	private static Path _toFeederStationFromFirstBayL;
	private static Path _toFirstBayFromFeederStation;
	private static Path _toSecondBayFromFeederStation;
	private static Path _toFrontCargoShipRFromR;
	
	public enum Right {
		TO_FRONT_CARGO_SHIP_R;
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
	
	private static void buildCenterPaths() {
		_toFrontCargoShipLFromC = buildPathFromWaypoints(Arrays.asList(
			new Waypoint(66,160,0,0),
			new Waypoint(115,160,20,20),
			new Waypoint(160,151,20,20),
			new Waypoint(185,151,0,20)
		));
		centerPaths.put(Center.TO_FRONT_CARGO_SHIP_L, _toFrontCargoShipLFromC);

	}
	
	private static void buildLeftPaths() {		
		_toFrontCargoShipLFromL = buildPathFromWaypoints(Arrays.asList(
		new Waypoint(66,120,0,0),
		new Waypoint(115,120,20,20),
		new Waypoint(145,160,20,20),
		new Waypoint(185,160,0,20)));
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
			new Waypoint(90,75,30,20),
			new Waypoint(20,75,0,20)
		));
		leftPaths.put(Left.FROM_FIRST_BAY_TO_FEEDER_STATION, _toFeederStationFromFirstBayL);

		_toFeederStationFromFrontL = buildPathFromWaypoints( Arrays.asList(
			new Waypoint(180,155,0,0),
			new Waypoint(130,30,35,20),
			new Waypoint(60,30,0,20)
		));
		leftPaths.put(Left.FROM_FRONT_CARGO_SHIP_L_TO_FEEDER_STATION, _toFeederStationFromFrontL);

		_toFirstBayFromFeederStation = buildPathFromWaypoints(Arrays.asList(
			new Waypoint(60,30,0,0),
			new Waypoint(150,60,20,20),
			new Waypoint(368,70,20,20),
			new Waypoint(368,100,0,20)
		));
		leftPaths.put(Left.FROM_FEEDER_STATION_TO_FIRST_BAY, _toFirstBayFromFeederStation);

		_toSecondBayFromFeederStation = buildPathFromWaypoints(Arrays.asList(
			new Waypoint(60,30,0,0),
			new Waypoint(150,60,20,20),
			new Waypoint(310,70,20,20),
			new Waypoint(310,110,0,20)
		));
		leftPaths.put(Left.FROM_FEEDER_STATION_TO_SECOND_BAY, _toSecondBayFromFeederStation);
	}
	
	private static void buildRightPaths() {
		_toFrontCargoShipRFromR = buildPathFromWaypoints(Arrays.asList(
			new Waypoint(66,204,0,0),
			new Waypoint(115,204,20,20),
			new Waypoint(145,160,20,20),
			new Waypoint(190,160,0,20)));
			rightPaths.put(Right.TO_FRONT_CARGO_SHIP_R, _toFrontCargoShipRFromR);
	}
}