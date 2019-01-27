/*----------------------------------------------------------------------------*/
/* Copyright (c) 2018 FIRST. All Rights Reserved.                             */
/* Open Source Software - may be modified and shared by FRC teams. The code   */
/* must be accompanied by the FIRST BSD license file in the root directory of */
/* the project.                                                               */
/*----------------------------------------------------------------------------*/

package frc.robot.subsystems;

import edu.wpi.first.wpilibj.command.Subsystem;
import frc.robot.interfaces.IBeakSquadSubsystem;
import frc.robot.util.LogDataBE;

/**
 * Hatch Subsystem
 *  Likely mostly a Pneumatic Subsystem
 * 
 * Student Lead: 
 */
public class Hatch extends Subsystem implements IBeakSquadSubsystem {

  //=====================================================================================
	// Define Singleton Pattern
	//=====================================================================================
	private static Hatch _instance = new Hatch();
	
	public static Hatch getInstance() {
		return _instance;
	}
	
	// private constructor for singleton pattern
	private Hatch() {
  }

  // Put methods for controlling this subsystem
  // here. Call these from Commands.

  @Override
  public void initDefaultCommand() {
    // Set the default command for a subsystem here.
    // setDefaultCommand(new MySpecialCommand());
  }

  //=====================================================================================
	// Helper Methods
  //=====================================================================================
  
  @Override
  public void updateLogData(LogDataBE logData) {

  }

  @Override
  public void updateDashboard() {

  }
}
