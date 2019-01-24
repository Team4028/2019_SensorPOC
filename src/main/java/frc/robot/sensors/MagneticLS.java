/*----------------------------------------------------------------------------*/
/* Copyright (c) 2018 FIRST. All Rights Reserved.                             */
/* Open Source Software - may be modified and shared by FRC teams. The code   */
/* must be accompanied by the FIRST BSD license file in the root directory of */
/* the project.                                                               */
/*----------------------------------------------------------------------------*/

package frc.robot.sensors;

import com.ctre.phoenix.motorcontrol.ControlMode;
import com.ctre.phoenix.motorcontrol.can.TalonSRX;

import edu.wpi.first.wpilibj.DigitalInput;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import frc.robot.RobotMap;

/**
 * This class exposes the OnBoard Analog Pressure Sensor Lead Student:
 */
public class MagneticLS 
{
	private DigitalInput magneticSwitch;
	private TalonSRX _lstalon;
    //=====================================================================================
	// Define Singleton Pattern
	//=====================================================================================
	private static MagneticLS _instance = new MagneticLS();
	
	public static MagneticLS getInstance() 
	{
		return _instance;
	}
	
	 //private constructor for singleton pattern
	private MagneticLS() 
	{
		magneticSwitch = new DigitalInput(RobotMap.REV_ROBOTICS_MAG_LIMIT);
		_lstalon = new TalonSRX(RobotMap.MAG_LS_TEST_TALON);
	}
	public void runTalonWithInputFromMagLimitSwitch(){
		if (!magneticSwitch.get())
		{
		_lstalon.set(ControlMode.PercentOutput, -0.1);
		}
	}
	public void putStatusofLsToSmartDash()
	{
		SmartDashboard.putBoolean("Status of Mag Limit Switch",magneticSwitch.get());
	}
}
