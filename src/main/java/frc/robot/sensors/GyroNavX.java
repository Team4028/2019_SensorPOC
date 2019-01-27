/*----------------------------------------------------------------------------*/
/* Copyright (c) 2018 FIRST. All Rights Reserved.                             */
/* Open Source Software - may be modified and shared by FRC teams. The code   */
/* must be accompanied by the FIRST BSD license file in the root directory of */
/* the project.                                                               */
/*----------------------------------------------------------------------------*/

package frc.robot.sensors;

import com.kauailabs.navx.frc.AHRS;

import edu.wpi.first.wpilibj.DriverStation;
import frc.robot.RobotMap;

/**
 * This class exposes the OnBoard Navigation Sensor Lead Student:
 */
public class GyroNavX {
    //=====================================================================================
	// Define Singleton Pattern
	//=====================================================================================
	// define class level working variables
	private AHRS _navXSensor;
	
	private VisionLL _visionLL = VisionLL.getInstance();
	
	private static GyroNavX _instance = new GyroNavX();
	
	public static GyroNavX getInstance() {
		return _instance;
	}
	
	// private constructor for singleton pattern
	private GyroNavX() {	
		try {          
			_navXSensor = new AHRS(RobotMap.NAVX_PORT); // Communication via RoboRIO MXP (SPI) 
		  } catch (RuntimeException ex ) {
			  DriverStation.reportError("Error instantiating navX MXP:  " + ex.getMessage(), true);
		  }
	}

	public double get_angle2InDegreesFromLL() {
		double angle2 =-90 - _visionLL.get_angle1InDegrees() - _navXSensor.getYaw();
		return angle2;
	}
	
    public double getYaw() { 
	return _navXSensor.getYaw();
	}
	
	public void zeroYaw() { 
		_navXSensor.zeroYaw(); 
	}
	
	public double getPitch() { //Axis Perpendicular to the Front/Back of the robot
	return _navXSensor.getPitch();
	}
}
