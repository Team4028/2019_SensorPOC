/*----------------------------------------------------------------------------*/
/* Copyright (c) 2018 FIRST. All Rights Reserved.                             */
/* Open Source Software - may be modified and shared by FRC teams. The code   */
/* must be accompanied by the FIRST BSD license file in the root directory of */
/* the project.                                                               */
/*----------------------------------------------------------------------------*/

package frc.robot.subsystems;

import com.ctre.phoenix.motorcontrol.ControlMode;
import com.ctre.phoenix.motorcontrol.can.TalonSRX;
import com.ctre.phoenix.motorcontrol.can.VictorSPX;

import edu.wpi.first.wpilibj.command.Subsystem;
import frc.robot.interfaces.IBeakSquadSubsystem;
import frc.robot.util.LogDataBE;
import frc.robot.RobotMap;

/**
 * Climber Subsystem
 *  1 Motor on CAN TalonSPX Motor Controller  w/ encoder + l/s?
 *  1 Motor on CAN VictorSRX Motor Controller open loop
 * 
 * Student Lead: Peter Nicholas
 */
public class Climber extends Subsystem implements IBeakSquadSubsystem {

  TalonSRX _liftMtr;
  VictorSPX _driveMtr;

  //=====================================================================================
	// Define Singleton Pattern
	//=====================================================================================
	private static Climber _instance = new Climber();
	
	public static Climber getInstance() {
		return _instance;
	}
	
	// private constructor for singleton pattern
	private Climber() {
    _liftMtr = new TalonSRX(RobotMap.CLIMBER_LIFT_CAN_ADDR);
    _liftMtr.configFactoryDefault();

    _driveMtr = new VictorSPX(RobotMap.CLIMBER_DRIVE_CAN_ADDR);
    _driveMtr.configFactoryDefault();
  }

  public void liftClimber(double liftSpeed){
    _liftMtr.set(ControlMode.PercentOutput, liftSpeed);
  }

  public void driveClimber(double driveSpeed){
    _driveMtr.set(ControlMode.PercentOutput, driveSpeed);
  }

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
