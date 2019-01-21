/*----------------------------------------------------------------------------*/
/* Copyright (c) 2018 FIRST. All Rights Reserved.                             */
/* Open Source Software - may be modified and shared by FRC teams. The code   */
/* must be accompanied by the FIRST BSD license file in the root directory of */
/* the project.                                                               */
/*----------------------------------------------------------------------------*/

package frc.robot.sensors;

/**
 * This class exposes the OnBoard Analog Pressure Sensor
 *  Lead Student: 
 */
public class StoredPressureSensor {
    
    //=====================================================================================
	// Define Singleton Pattern
	//=====================================================================================
	private static StoredPressureSensor _instance = new StoredPressureSensor();
	
	public static StoredPressureSensor getInstance() {
		return _instance;
	}
	
	// private constructor for singleton pattern
	private StoredPressureSensor() 
	{	
	}

	public double get_storedPSI()
	{
		return 0.0;
	}
}
