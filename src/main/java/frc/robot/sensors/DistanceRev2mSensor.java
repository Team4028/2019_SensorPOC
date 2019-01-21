/*----------------------------------------------------------------------------*/
/* Copyright (c) 2018 FIRST. All Rights Reserved.                             */
/* Open Source Software - may be modified and shared by FRC teams. The code   */
/* must be accompanied by the FIRST BSD license file in the root directory of */
/* the project.                                                               */
/*----------------------------------------------------------------------------*/

package frc.robot.sensors;

import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import frc.robot.interfaces.IDistanceSensor;
import frc.robot.sensors.revSrc.DistanceUnit;
import frc.robot.sensors.revSrc.VL53L0X;
import frc.robot.util.LogDataBE;
import frc.robot.RobotMap;
import frc.robot.util.GeneralUtilities;
/**
 * This class exposes the onboard Distance Sensor Lead Student:
 */
public class DistanceRev2mSensor implements IDistanceSensor{

	private VL53L0X _distanceSensor;

    //=====================================================================================
	// Define Singleton Pattern
	//=====================================================================================
	private static DistanceRev2mSensor _instance = new DistanceRev2mSensor();
	
	public static DistanceRev2mSensor getInstance() {
		return _instance;
	}

	// private constructor for singleton pattern
	private DistanceRev2mSensor() {	
		_distanceSensor = new VL53L0X(RobotMap.I2C_SENSOR_PORT, 0x29);
		SmartDashboard.putBoolean("VL53LOX:isSensorPresent", get_isSensorPresent());
	}

	public void updateDashboard(){
		SmartDashboard.putNumber("VL53LOX:DistanceInInches", get_distanceToTargetInInches());
		SmartDashboard.putBoolean("VL53LOX:didTimeoutOccur", get_didTimeoutOccur());
		
	}
	
  	public void updateLogData(LogDataBE logData) {
	}

	@Override
	public double get_distanceToTargetInInches() {
		return GeneralUtilities.roundDouble(_distanceSensor.getDistance(DistanceUnit.INCH),2);
	}

	public boolean get_didTimeoutOccur(){
		return _distanceSensor.didTimeoutOccur();
	} 
	public boolean get_isSensorPresent(){
		return _distanceSensor.doInitialize();
	} 
}