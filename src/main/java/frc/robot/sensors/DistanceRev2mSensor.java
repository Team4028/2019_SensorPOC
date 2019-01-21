/*----------------------------------------------------------------------------*/
/* Copyright (c) 2018 FIRST. All Rights Reserved.                             */
/* Open Source Software - may be modified and shared by FRC teams. The code   */
/* must be accompanied by the FIRST BSD license file in the root directory of */
/* the project.                                                               */
/*----------------------------------------------------------------------------*/

package frc.robot.sensors;

import frc.robot.interfaces.IDistanceSensor;

/**
 * This class exposes the onboard Distance Sensor Lead Student:
 */
public class DistanceRev2mSensor implements IDistanceSensor {
    
    //=====================================================================================
	// Define Singleton Pattern
	//=====================================================================================
	private static DistanceRev2mSensor _instance = new DistanceRev2mSensor();
	
	public static DistanceRev2mSensor getInstance() {
		return _instance;
	}
	
	// private constructor for singleton pattern
	private DistanceRev2mSensor() 
	{	
	}

	public double get_distanceToTargetInInches()
	{
		return 0.0;
	}
}