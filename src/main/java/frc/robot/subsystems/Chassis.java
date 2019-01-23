/*----------------------------------------------------------------------------*/
/* Copyright (c) 2018 FIRST. All Rights Reserved.                             */
/* Open Source Software - may be modified and shared by FRC teams. The code   */
/* must be accompanied by the FIRST BSD license file in the root directory of */
/* the project.                                                               */
/*----------------------------------------------------------------------------*/

package frc.robot.subsystems;

import com.ctre.phoenix.motorcontrol.ControlMode;
import com.ctre.phoenix.motorcontrol.NeutralMode;
import com.ctre.phoenix.motorcontrol.can.TalonSRX;

import edu.wpi.first.wpilibj.command.Subsystem;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import frc.robot.RobotMap;
import frc.robot.interfaces.IBeakSquadSubsystem;
import frc.robot.sensors.GyroNavX;
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
  
  public enum ChassisState
	{
		UNKNOWN,
		PERCENT_VBUS,
		AUTO_TURN, 
		FOLLOW_PATH,
		DRIVE_SET_DISTANCE
  }
  
  ChassisState _chassisState = ChassisState.UNKNOWN;
  GyroNavX _navX = GyroNavX.getInstance();

  TalonSRX _leftMaster, _leftSlave, _rightMaster, _rightSlave;



  double _targetAngle,_angleError;
  double ENCODER_CODES_PER_DEGREE = 339.2468;
  boolean _isTurnRight;
	public static Chassis getInstance() {
		return _instance;
	}
	
	// private constructor for singleton pattern
  private Chassis() 
  {
    _leftMaster = new TalonSRX(RobotMap.LEFT_DRIVE_MASTER_CAN_ADDR);
    _leftSlave = new TalonSRX(RobotMap.LEFT_DRIVE_SLAVE_CAN_ADDR);
    _rightMaster = new TalonSRX(RobotMap.RIGHT_DRIVE_MASTER_CAN_ADDR);
    _rightSlave = new TalonSRX(RobotMap.RIGHT_DRIVE_SLAVE_CAN_ADDR);

    _leftSlave.follow(_leftMaster);
    _rightSlave.follow(_rightMaster);
    _rightMaster.setInverted(false);
    _rightSlave.setInverted(false);
    _leftMaster.setInverted(true);
    _leftSlave.setInverted(true);

    _leftMaster.setNeutralMode(NeutralMode.Brake);
		_leftSlave.setNeutralMode(NeutralMode.Brake);
		_rightMaster.setNeutralMode(NeutralMode.Brake);
		_rightSlave.setNeutralMode(NeutralMode.Brake);
  }

  @Override
  public void initDefaultCommand() {
    // Set the default command for a subsystem here.
    // setDefaultCommand(new MySpecialCommand());
  }

  // basic driving
  public void arcadeDrive(double throttleCmd, double turnCmd) 
  {
    _leftMaster.set(ControlMode.PercentOutput, throttleCmd+0.7*turnCmd);
    _rightMaster.set(ControlMode.PercentOutput, throttleCmd-0.7*turnCmd);
  }

  public void updateChassis(double timestamp){
		switch(_chassisState) {
			case UNKNOWN:
			return;
			case PERCENT_VBUS:
				return;
				
			case AUTO_TURN:
        _leftMaster.config_kF(0, 0);
        _leftMaster.config_kP(0, 0);
        _leftMaster.config_kI(0, 0);
        _leftMaster.config_kD(0, 0);
        _rightMaster.config_kF(0, 0);
        _rightMaster.config_kP(0, 0);
        _rightMaster.config_kI(0, 0);
        _rightMaster.config_kD(0, 0);
				moveToTargetAngle();
				return;
			/*	
			case DRIVE_SET_DISTANCE:
				GeneralUtilities.setPIDFGains(_leftMaster, MOTION_MAGIC_STRAIGHT_PIDF_GAINS);
				GeneralUtilities.setPIDFGains(_rightMaster, MOTION_MAGIC_STRAIGHT_PIDF_GAINS);
				moveToTargetPosDriveSetDistance();
				return;
				
			case FOLLOW_PATH:
				if (get_isHighGear()) {
					GeneralUtilities.setPIDFGains(_leftMaster, HIGH_GEAR_VELOCITY_PIDF_GAINS);
					GeneralUtilities.setPIDFGains(_rightMaster, HIGH_GEAR_VELOCITY_PIDF_GAINS);
				} else {
					GeneralUtilities.setPIDFGains(_leftMaster, LOW_GEAR_VELOCITY_PIDF_GAINS);
					GeneralUtilities.setPIDFGains(_rightMaster, LOW_GEAR_VELOCITY_PIDF_GAINS);
				}
				
				if (_pathFollower != null) 
					updatePathFollower(timestamp);
				return;*/
		}
	}

  public synchronized void setTargetAngleAndTurnDirection(double targetAngle, boolean isTurnRight) 
	{
		_targetAngle = targetAngle;
		_isTurnRight = isTurnRight;
		_chassisState = ChassisState.AUTO_TURN;
  }
  
  public void moveToTargetAngle()
    {
        if((!_isTurnRight && getHeading() > _targetAngle) || (_isTurnRight && getHeading() < _targetAngle))
        {
            _angleError = _targetAngle - getHeading();
        }           
        else if(!_isTurnRight && getHeading() < _targetAngle)
        {
            _angleError = _targetAngle - getHeading() - 360;
        }
        else if(_isTurnRight && getHeading() > _targetAngle)
        {
            _angleError = 360 - getHeading() + _targetAngle;
        }

        //double encoderError = ENCODER_CODES_PER_DEGREE * _angleError;       
       // double leftDriveTargetPos = getLeftPos() + encoderError;
       // double rightDriveTargetPos = getRightPos() - encoderError;
        
        //setLeftRightCommand(ControlMode.MotionMagic, leftDriveTargetPos, rightDriveTargetPos);
    }
	//=====================================================================================
	// Helper Methods
  //=====================================================================================
  public void setLeftRightCommand(ControlMode mode, double leftCommand, double rightCommand) 
  {
		_leftMaster.set(mode, leftCommand);
		_rightMaster.set(mode, rightCommand);
	}
  public double getHeading()
  {
    if(_navX.getYaw()<0)
    {
      return 360+_navX.getYaw();
    }
    else
    {
      return _navX.getYaw();
    }
  }

  public double getLeftPos()
	{
		return _leftMaster.getSelectedSensorPosition(0);
	}
	public double getRightPos()
	{
		return _rightMaster.getSelectedSensorPosition(0);
  }
  public void zeroSensors()
  {
    _leftMaster.setSelectedSensorPosition(0);
    _rightMaster.setSelectedSensorPosition(0);
    _navX.zeroYaw();
  }
  
  @Override
  public void updateLogData(LogDataBE logData) 
  {

  }

  @Override
  public void updateDashboard() 
  {

  }


}