/*----------------------------------------------------------------------------*/
/* Copyright (c) 2018 FIRST. All Rights Reserved.                             */
/* Open Source Software - may be modified and shared by FRC teams. The code   */
/* must be accompanied by the FIRST BSD license file in the root directory of */
/* the project.                                                               */
/*----------------------------------------------------------------------------*/

package frc.robot.sensors;

/**
 * This class exposes the OnBoard Navigation Sensor
 *  Lead Student: 
 */
public class GyroNavX {
    //=====================================================================================
	// Define Singleton Pattern
	//=====================================================================================
	private static GyroNavX _instance = new GyroNavX();
	
	public static GyroNavX getInstance() {
		return _instance;
	}
	
	// private constructor for singleton pattern
	private GyroNavX() 
	{	
    }
}
