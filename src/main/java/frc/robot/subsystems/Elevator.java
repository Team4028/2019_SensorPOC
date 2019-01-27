/*----------------------------------------------------------------------------*/
/* Copyright (c) 2018 FIRST. All Rights Reserved.                             */
/* Open Source Software - may be modified and shared by FRC teams. The code   */
/* must be accompanied by the FIRST BSD license file in the root directory of */
/* the project.                                                               */
/*----------------------------------------------------------------------------*/

package frc.robot.subsystems;

import com.ctre.phoenix.motorcontrol.can.TalonSRX;

import edu.wpi.first.wpilibj.command.Subsystem;
import frc.robot.interfaces.IBeakSquadSubsystem;
import frc.robot.util.LogDataBE;

/**
 * Elevator Subsystem
 *  Master + Slave Motors on CAN TalonSRX Motor Controllers
 *  Master Motor will have:
 *    - Top & Bottom Limit Switches (likley new mag limit switch)
 *    - Quad Encoder
 * 
 * Student Lead: 
 */
public class Elevator extends Subsystem implements IBeakSquadSubsystem {

  TalonSRX _masterMtr;
  TalonSRX _slaveMtr;

  //=====================================================================================
	// Define Singleton Pattern
	//=====================================================================================
	private static Elevator _instance = new Elevator();
	
	public static Elevator getInstance() {
		return _instance;
	}
	
	// private constructor for singleton pattern
	private Elevator() {
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
