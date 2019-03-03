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
		TO_FRONT_CARGO_SHIP_R;
	}

	private static Path _toFrontCargoShipLFromC;
	
	public enum Left {
		TO_FRONT_CARGO_SHIP_L,
		TO_LEFT_CARGO_SHIP_FIRST,
		FROM_FIRST_BAY_TO_FEEDER_STATION,
		FROM_FRONT_CARGO_SHIP_L_TO_FEEDER_STATION,
		FROM_FEEDER_STATION_TO_FIRST_BAY,
		FROM_FEEDER_STATION_TO_SECOND_BAY,
		AWAY_FROM_FEEDER,
		BACK_ROCKET
	}
	
	private static Path _toFrontCargoShipLFromL;
	private static Path _toLeftCargoShipFirstBay;
	private static Path _toFeederStationFromFrontL;
	private static Path _toFeederStationFromFirstBayL;
	private static Path _toFirstBayFromFeederStationL;
	private static Path _toSecondBayFromFeederStationL;
	private static Path _toFrontCargoShipRFromR;
	private static Path _toRightCargoShipFirstBay;
	private static Path _toCargoShipRFromC;
	private static Path _toFeederStationFromFirstBayR;
	private static Path _toFeederStationFromFrontR;
	private static Path _toSecondBayFromFeederStationR;
	private static Path _toFirstBayFromFeederStationR;
	private static Path _awayFromFeederStationL;
	private static Path _toBackRocketL;
	private static Path _awayFromBackRocketL;
	
	public enum Right {
		TO_FRONT_CARGO_SHIP_R,
		TO_RIGHT_CARGO_BAY_FIRST,
		TO_FEEDER_STATION_FROM_FIRST_BAY,
		TO_FEEDER_STATION_FROM_R_FRONT,
		TO_SECOND_BAY_FROM_FEEDER_STATION,
		TO_FIRST_BAY_FROM_FEEDER_STATION
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
			new Waypoint(115,160,15,40),
			new Waypoint(145,151,17,80),
			new Waypoint(185,151,0,40)
		));
		centerPaths.put(Center.TO_FRONT_CARGO_SHIP_L, _toFrontCargoShipLFromC);
		_toCargoShipRFromC = buildPathFromWaypoints(-0.001,Arrays.asList(
			new Waypoint(66,164,0,0),
			new Waypoint(115,164,25,30),
			new Waypoint(160,172,22,60),
			new Waypoint(185,172,0,60)
		));
		centerPaths.put(Center.TO_FRONT_CARGO_SHIP_R, _toCargoShipRFromC);

	}
	
	private static void buildLeftPaths() {		
		_toFrontCargoShipLFromL = buildPathFromWaypoints(-0.001,Arrays.asList(
		new Waypoint(66,120,0,0),
		new Waypoint(120,120,20,40),
		new Waypoint(145,157,20,70),
		new Waypoint(185,157,0,30)));
		leftPaths.put(Left.TO_FRONT_CARGO_SHIP_L, _toFrontCargoShipLFromL);

		_toLeftCargoShipFirstBay = buildPathFromWaypoints(Arrays.asList(
			new Waypoint(66,120,0,0),
			new Waypoint(115,120,40,50),
			new Waypoint(265,100,0,120)
		));
		leftPaths.put(Left.TO_LEFT_CARGO_SHIP_FIRST, _toLeftCargoShipFirstBay);

		_toFeederStationFromFirstBayL = buildPathFromWaypoints(-0.005,Arrays.asList(
			new Waypoint(260,110,0,0),
			new Waypoint(130,85,60,100),
			new Waypoint(30,85,0,120)
		));
		leftPaths.put(Left.FROM_FIRST_BAY_TO_FEEDER_STATION, _toFeederStationFromFirstBayL);

		_toFeederStationFromFrontL = buildPathFromWaypoints(-0.001, Arrays.asList(
			new Waypoint(180,155,0,0),
			new Waypoint(120,69,0,120),
			new Waypoint(90,28,30,60),
			new Waypoint(45,28,0,100)
		
		));
		leftPaths.put(Left.FROM_FRONT_CARGO_SHIP_L_TO_FEEDER_STATION, _toFeederStationFromFrontL);

		_toFirstBayFromFeederStationL = buildPathFromWaypoints(Arrays.asList(
			new Waypoint(60,30,0,0),
			new Waypoint(150,60,20,20),
			new Waypoint(368,70,20,20),
			new Waypoint(368,100,0,20)
		));
		leftPaths.put(Left.FROM_FEEDER_STATION_TO_FIRST_BAY, _toFirstBayFromFeederStationL);

		_toSecondBayFromFeederStationL = buildPathFromWaypoints(Arrays.asList(
			new Waypoint(60,30,0,0),
			new Waypoint(150,60,20,20),
			new Waypoint(315,70,20,20),
			new Waypoint(315,110,0,20)
		));
		leftPaths.put(Left.FROM_FEEDER_STATION_TO_SECOND_BAY, _toSecondBayFromFeederStationL);

		_awayFromFeederStationL = buildPathFromWaypoints(Arrays.asList(
			new Waypoint(30,85,0,0),
			new Waypoint(100,85,50,80),
			new Waypoint(150,110,0,120)
		));
		_awayFromFeederStationL.setIsReversed(true);
		leftPaths.put(Left.AWAY_FROM_FEEDER, _awayFromFeederStationL);

		_toBackRocketL= buildPathFromWaypoints(-0.005,Arrays.asList(
			new Waypoint(66,120,0,0),
			new Waypoint(115,120,0,50),
			new Waypoint(180,120, 40, 80),
			new Waypoint(210,93,0,90),
			new Waypoint(268,38,20,70),
			new Waypoint(293,45,0,40)
		));
		_toBackRocketL.setIsReversed(true);
		leftPaths.put(Left.BACK_ROCKET, _toBackRocketL);
		

	}
	
	private static void buildRightPaths() {
		_toFrontCargoShipRFromR = buildPathFromWaypoints(Arrays.asList(
			new Waypoint(66,204,0,0),
			new Waypoint(115,204,20,20),
			new Waypoint(145,159,20,20),
			new Waypoint(185,159,0,20)));
			rightPaths.put(Right.TO_FRONT_CARGO_SHIP_R, _toFrontCargoShipRFromR);
		_toRightCargoShipFirstBay = buildPathFromWaypoints(Arrays.asList(
			new Waypoint(66,204,0,0),
			new Waypoint(130,204,20,20),
			new Waypoint(160,244,20,20),
			new Waypoint(256,244,16,20),
			new Waypoint(256,224,0,20)
		));
		rightPaths.put(Right.TO_RIGHT_CARGO_BAY_FIRST, _toRightCargoShipFirstBay);
		_toFeederStationFromFirstBayR = buildPathFromWaypoints(Arrays.asList(
			new Waypoint(256,229,0,0),
			new Waypoint(234,239,16,20),
			new Waypoint(155,289,30,20),
			new Waypoint(86,345,18,20),
			new Waypoint(65,345,0,20)
		));
		rightPaths.put(Right.TO_FEEDER_STATION_FROM_FIRST_BAY, _toFeederStationFromFirstBayR);

		_toSecondBayFromFeederStationR = buildPathFromWaypoints(Arrays.asList(
			new Waypoint(60,294,0,0),
			new Waypoint(150,264,20,20),
			new Waypoint(310,254,20,20),
			new Waypoint(310,214,0,20)
		));
		rightPaths.put(Right.TO_SECOND_BAY_FROM_FEEDER_STATION, _toSecondBayFromFeederStationR);

		_toFeederStationFromFrontR = buildPathFromWaypoints( Arrays.asList(
			new Waypoint(180,169,0,0),
			new Waypoint(130,280,35,20),
			new Waypoint(20,280,0,20)
		));
		rightPaths.put(Right.TO_FEEDER_STATION_FROM_R_FRONT, _toFeederStationFromFrontR);

		_toFirstBayFromFeederStationR = buildPathFromWaypoints(Arrays.asList(
			new Waypoint(60,294,0,0),
			new Waypoint(150,264,20,20),
			new Waypoint(210,254,20,20),
			new Waypoint(210,224,0,20)
		));
		rightPaths.put(Right.TO_FIRST_BAY_FROM_FEEDER_STATION, _toFirstBayFromFeederStationR);
	}
}