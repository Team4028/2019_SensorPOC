package frc.robot.subsystems;

import com.ctre.phoenix.motorcontrol.can.TalonSRX;
import com.ctre.phoenix.motorcontrol.can.VictorSPX;

import edu.wpi.first.wpilibj.command.Subsystem;
import frc.robot.interfaces.IBeakSquadSubsystem;
import frc.robot.util.LogDataBE;

/**
 * Climber Subsystem
 *  1 Motor on CAN TalonSPX Motor Controller  w/ encoder + l/s?
 *  1 Motor on CAN VictorSRX Motor Controller open loop
 * 
 * Student Lead: 
 */
public class Climber extends Subsystem implements IBeakSquadSubsystem {
	// =================================================================================================================
  // define class level working variables
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
	private Climber() {}

	// ===============================================================================================================
	// Default Command
	// ===============================================================================================================
  @Override
  public void initDefaultCommand() {}

	// ===============================================================================================================
	// General Purpose Utility Methods
	// ===============================================================================================================
	@Override
  public void updateLogData(LogDataBE logData) {}

  @Override
  public void updateDashboard() {}
}
