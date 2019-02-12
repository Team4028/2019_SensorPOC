/*----------------------------------------------------------------------------*/
/* Copyright (c) 2018 FIRST. All Rights Reserved.                             */
/* Open Source Software - may be modified and shared by FRC teams. The code   */
/* must be accompanied by the FIRST BSD license file in the root directory of */
/* the project.                                                               */
/*----------------------------------------------------------------------------*/

package frc.robot.subsystems;

import com.ctre.phoenix.motorcontrol.ControlMode;
import com.ctre.phoenix.motorcontrol.FeedbackDevice;
import com.ctre.phoenix.motorcontrol.LimitSwitchNormal;
import com.ctre.phoenix.motorcontrol.LimitSwitchSource;
import com.ctre.phoenix.motorcontrol.NeutralMode;
import com.ctre.phoenix.motorcontrol.StatusFrame;
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

    //Configure Limit Switches
    /*_liftMtr.configForwardLimitSwitchSource(LimitSwitchSource.FeedbackConnector, LimitSwitchNormal.NormallyClosed);
    _liftMtr.configReverseLimitSwitchSource(LimitSwitchSource.FeedbackConnector, LimitSwitchNormal.NormallyClosed);

    // Turn of all soft limits
   _liftMtr.configForwardSoftLimitEnable(false, 0);
   _liftMtr.configReverseSoftLimitEnable(false, 0);

    //Configure brake mode
   _liftMtr.setNeutralMode(NeutralMode.Brake);
  _liftMtr.setNeutralMode(NeutralMode.Brake);

    // Configure Encoder
   _liftMtr.configSelectedFeedbackSensor(FeedbackDevice.CTRE_MagEncoder_Relative, 0, 0);
   _liftMtr.setSensorPhase(true);
   _liftMtr.setStatusFramePeriod(StatusFrame.Status_2_Feedback0, 5, 0);*/


    _driveMtr = new VictorSPX(RobotMap.CLIMBER_DRIVE_CAN_ADDR);
    _driveMtr.configFactoryDefault();
  }

  public void liftClimber(double liftSpeed){
    _liftMtr.set(ControlMode.PercentOutput, .25 * liftSpeed);
  }

  public void driveClimber(double driveSpeed){
    _driveMtr.set(ControlMode.PercentOutput, .25 * driveSpeed);
  }

  public double nativeUnitsToInches(){
    return _liftMtr.getSelectedSensorPosition() * .0013729128051576489005976;
  }

  public double getNativeUnits(){
    return ( _liftMtr).getSelectedSensorPosition();
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
