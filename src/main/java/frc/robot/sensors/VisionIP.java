/*----------------------------------------------------------------------------*/
/* Copyright (c) 2018 FIRST. All Rights Reserved.                             */
/* Open Source Software - may be modified and shared by FRC teams. The code   */
/* must be accompanied by the FIRST BSD license file in the root directory of */
/* the project.                                                               */
/*----------------------------------------------------------------------------*/

package frc.robot.sensors;

import frc.robot.interfaces.IVisionSensor;

/**
 * This class exposes the OnBoard IPhone Vision sensor
 *  Lead Student: 
 */
public class VisionIP implements IVisionSensor {

    //=====================================================================================
	// Define Singleton Pattern
	//=====================================================================================
	private static VisionIP _instance = new VisionIP();
	
	public static VisionIP getInstance() {
		return _instance;
	}
	
	// private constructor for singleton pattern
	private VisionIP() 
	{	
    }

    @Override
    public double get_angle1InDegrees() {
        return 0;
    }

    @Override
    public double get_distanceToTargetInInches() {
        return 0;
    }

}
