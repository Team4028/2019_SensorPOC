/*----------------------------------------------------------------------------*/
/* Copyright (c) 2018 FIRST. All Rights Reserved.                             */
/* Open Source Software - may be modified and shared by FRC teams. The code   */
/* must be accompanied by the FIRST BSD license file in the root directory of */
/* the project.                                                               */
/*----------------------------------------------------------------------------*/

package frc.robot.subsystems;

import com.ctre.phoenix.motorcontrol.can.VictorSPX;

import edu.wpi.first.wpilibj.command.Subsystem;
import frc.robot.interfaces.IBeakSquadSubsystem;
import frc.robot.util.LogDataBE;

/**
 * Cargo (Ball) Subsystem
 *  Motors on CAN VictorSRX Motor Controllers
 *  Motor will primarily run in vbus mode to infeed & outfeed the ball
 * 
 * Student Lead: 
 */
public class Cargo extends Subsystem implements IBeakSquadSubsystem {

  VictorSPX _infeedMtr;

  //=====================================================================================
	// Define Singleton Pattern
	//=====================================================================================
	private static Cargo _instance = new Cargo();
	
	public static Cargo getInstance() {
		return _instance;
	}
	
	// private constructor for singleton pattern
	private Cargo() {
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
