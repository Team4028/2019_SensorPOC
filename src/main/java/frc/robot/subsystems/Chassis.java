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
 * Add your docs here.
 */
public class Chassis extends Subsystem implements IBeakSquadSubsystem {
  // Put methods for controlling this subsystem
  // here. Call these from Commands.

	//=====================================================================================
	// Define Singleton Pattern
	//=====================================================================================
	private static Chassis _instance = new Chassis();
	
	public static Chassis getInstance() {
		return _instance;
	}
	
	// private constructor for singleton pattern
	private Chassis() {}

  @Override
  public void initDefaultCommand() {}

  // basic driving
  public void arcadeDrive(double throttleCmd, double turnCmd) {}

	//=====================================================================================
	// Helper Methods
	//=====================================================================================
  @Override
  public void updateLogData(LogDataBE logData) {}

  @Override
  public void updateDashboard() {}
}