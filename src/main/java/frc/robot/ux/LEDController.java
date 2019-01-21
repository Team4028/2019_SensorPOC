/*----------------------------------------------------------------------------*/
/* Copyright (c) 2018 FIRST. All Rights Reserved.                             */
/* Open Source Software - may be modified and shared by FRC teams. The code   */
/* must be accompanied by the FIRST BSD license file in the root directory of */
/* the project.                                                               */
/*----------------------------------------------------------------------------*/

package frc.robot.ux;

/**
 * This class controls the onboard LEDs
 *  Lead Student: 
 */
public class LEDController {

    //=====================================================================================
	// Define Singleton Pattern
	//=====================================================================================
	private static LEDController _instance = new LEDController();
	
	public static LEDController getInstance() {
		return _instance;
	}
	
	// private constructor for singleton pattern
	private LEDController() 
	{	
	}
	
	// call this method to display the correct LED Color
	public void set_distance(double currentDistanceInInches)
	{

	}
}
