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
import com.ctre.phoenix.motorcontrol.VelocityMeasPeriod;
import com.ctre.phoenix.motorcontrol.can.TalonSRX;

import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj.command.Subsystem;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import frc.robot.Constants;
import frc.robot.RobotMap;
import frc.robot.auton.pathfollowing.RobotState;
import frc.robot.auton.pathfollowing.control.Path;
import frc.robot.auton.pathfollowing.control.PathFollower;
import frc.robot.auton.pathfollowing.motion.RigidTransform;
import frc.robot.auton.pathfollowing.motion.Rotation;
import frc.robot.auton.pathfollowing.motion.Twist;
import frc.robot.auton.pathfollowing.util.Kinematics;
import frc.robot.commands.auton.adaptivePaths.Auton_turnFromVision;
import frc.robot.interfaces.IBeakSquadSubsystem;
import frc.robot.sensors.GyroNavX;
import frc.robot.sensors.VisionLL;
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
  
  double ENCODER_CODES_PER_DEGREE = 46.21; //value calculated theoretically from ECPWR //339.2468*100/169.8;

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
  PathFollower _pathFollower=null;
  public TalonSRX _leftMaster, _leftSlave, _rightMaster, _rightSlave;

  public double _leftMtrDriveSetDistanceCmd, _rightMtrDriveSetDistanceCmd;

  public double _targetAngle,_angleError;
  VisionLL _limeLight = VisionLL.getInstance();
  boolean _isTurnRight;
  boolean _isVisionTargetVisible;

  double _leftTargetVelocity, _rightTargetVelocity, _centerTargetVelocity;
  Path _currentPath=null;
  RobotState _robotState = RobotState.getInstance();
  double _leftEncoderPrevDistance, _rightEncoderPrevDistance=0;

  public static double _autoStartTime;

  public static double ENCODER_COUNTS_PER_WHEEL_REV = 4008;
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
    _rightMaster.setInverted(true);
    _rightSlave.setInverted(true);
    _leftMaster.setInverted(false);
    _leftSlave.setInverted(false);

    _rightMaster.setSensorPhase(true);
    _leftMaster.setSensorPhase(true);

    _leftMaster.setNeutralMode(NeutralMode.Brake);
		_leftSlave.setNeutralMode(NeutralMode.Brake);
		_rightMaster.setNeutralMode(NeutralMode.Brake);
    _rightSlave.setNeutralMode(NeutralMode.Brake);
    
    configMasterMotors(_leftMaster);
		configMasterMotors(_rightMaster);
    
    configDriveMotors(_leftMaster);
    configDriveMotors(_rightMaster);
    configDriveMotors(_leftSlave);
		configDriveMotors(_rightSlave);
  }
  private void configMasterMotors(TalonSRX talon) {
		talon.configSelectedFeedbackSensor(FeedbackDevice.CTRE_MagEncoder_Relative, 0, 10);
		talon.setStatusFramePeriod(StatusFrame.Status_2_Feedback0, 5, 10);
	
        talon.configVelocityMeasurementPeriod(VelocityMeasPeriod.Period_10Ms, 0);
        talon.configVelocityMeasurementWindow(32, 0);
        

		talon.configClosedloopRamp(0.0, 0);
	}
	
	private void configDriveMotors(TalonSRX talon) 
	{
    //talon.configFactoryDefault();
		talon.configForwardLimitSwitchSource(LimitSwitchSource.Deactivated, LimitSwitchNormal.Disabled, 0);
		talon.configReverseLimitSwitchSource(LimitSwitchSource.Deactivated, LimitSwitchNormal.Disabled, 0);
        
        talon.enableCurrentLimit(false);
        talon.configOpenloopRamp(1, 10);
        
        talon.configPeakOutputForward(1.0, 10);
        talon.configPeakOutputReverse(-1.0, 10);
        talon.configNominalOutputForward(0, 10);
        talon.configNominalOutputReverse(0, 10);
        talon.configContinuousCurrentLimit(Constants.BIG_NUMBER, 10);
	}

  @Override
  public void initDefaultCommand() {}



  public void updateChassis(double timestamp){
    estimateRobotState(timestamp);
    if ( _chassisState != ChassisState.FOLLOW_PATH){
      if(Math.abs(_navX.getPitch())>13 && Math.abs(_navX.getPitch())<20)
      {
        stop();
      }
      else if(_navX.getPitch()>20)
      {
        _chassisState=ChassisState.PERCENT_VBUS;
        setLeftRightCommand(ControlMode.PercentOutput, 0.3, 0.3);
      }
      else if(_navX.getPitch()<-20)
      {
        _chassisState=ChassisState.PERCENT_VBUS;
        setLeftRightCommand(ControlMode.PercentOutput, -0.3, -0.3);
      }
    }
		switch(_chassisState) {
			case UNKNOWN:
			return;
      case PERCENT_VBUS:
				return;
      case AUTO_TURN:
        _leftMaster.config_kF(0, 0.4);
        _leftMaster.config_kP(0, 0.2);
        _leftMaster.config_kI(0, 0);
        _leftMaster.config_kD(0,8);
        _rightMaster.config_kF(0, 0.4);
        _rightMaster.config_kP(0, 0.2);
        _rightMaster.config_kI(0, 0);
        _rightMaster.config_kD(0, 8);
        _rightMaster.configMotionCruiseVelocity(1400);
        _leftMaster.configMotionCruiseVelocity(1400);
        _rightMaster.configMotionAcceleration(1200);
        _leftMaster.configMotionAcceleration(1200);
        return;
      case DRIVE_SET_DISTANCE:
        _leftMaster.config_kF(0, 0.802);
        _leftMaster.config_kP(0, 0.32);
        _leftMaster.config_kI(0, 0);
        _leftMaster.config_kD(0, 3.2);
        _rightMaster.config_kF(0, 0.802);
        _rightMaster.config_kP(0, 0.32);
        _rightMaster.config_kI(0, 0);
        _rightMaster.config_kD(0, 3.2);
        _rightMaster.configMotionCruiseVelocity(1275);
        _leftMaster.configMotionCruiseVelocity(1275);
        _rightMaster.configMotionAcceleration(900);
        _leftMaster.configMotionAcceleration(900);
				return;
				
      case FOLLOW_PATH:
        estimateRobotState(timestamp);
        _leftMaster.config_kF(0, 0.401);
        _leftMaster.config_kP(0, .8);
        _leftMaster.config_kI(0, 0.000); //.001
        _leftMaster.config_kD(0, 8);
        _rightMaster.config_kF(0, 0.401);
        _rightMaster.config_kP(0, 0.8);
        _rightMaster.config_kI(0, 0.000); //.001
        _rightMaster.config_kD(0, 8);
				if (_pathFollower != null) 
					updatePathFollower(timestamp);
				return;
		}
  }
  
    // basic driving
    public void arcadeDrive(double throttleCmd, double turnCmd) 
    {
      _chassisState = ChassisState.PERCENT_VBUS;
      if(turnCmd>0.3)
      {
        _rightMaster.configOpenloopRamp(0.3);
        _rightSlave.configOpenloopRamp(0.3);
        _leftMaster.configOpenloopRamp(0.3);
        _leftSlave.configOpenloopRamp(0.3);
      }
      else
      {
        _rightMaster.configOpenloopRamp(0.7);
        _rightSlave.configOpenloopRamp(0.7);
        _leftMaster.configOpenloopRamp(0.7);
        _leftSlave.configOpenloopRamp(0.7);
      }
      if(throttleCmd>0.5)
      {
        _leftMaster.set(ControlMode.PercentOutput, 0.7*throttleCmd+0.3*turnCmd);
        _rightMaster.set(ControlMode.PercentOutput,0.7*throttleCmd-0.3*turnCmd);
      }
      else
      {
        _leftMaster.set(ControlMode.PercentOutput, 0.7*throttleCmd+0.3*turnCmd);
        _rightMaster.set(ControlMode.PercentOutput,0.78*throttleCmd-0.3*turnCmd);
      }
    }
  
    public void stop()
    {
      _chassisState = ChassisState.PERCENT_VBUS;
      setLeftRightCommand(ControlMode.PercentOutput, 0,0);
      System.out.println("Chassis Stop");
    }



    public void setMotionMagicCmdInches(double Distance)
	{

		_leftMtrDriveSetDistanceCmd = _leftMaster.getSelectedSensorPosition(0)+ InchestoNU(Distance);
    _rightMtrDriveSetDistanceCmd = _rightMaster.getSelectedSensorPosition(0)+InchestoNU(Distance);
    _chassisState=ChassisState.DRIVE_SET_DISTANCE;
  }
  
  public void moveToTargetPosDriveSetDistance ()
	{
    _chassisState = ChassisState.DRIVE_SET_DISTANCE;
    setLeftRightCommand(ControlMode.MotionMagic, _leftMtrDriveSetDistanceCmd, _rightMtrDriveSetDistanceCmd);
  }
  
  public synchronized void setWantDrivePath(Path path, boolean reversed) 
	{
		if (_currentPath != path || _chassisState != ChassisState.FOLLOW_PATH) 
		{
			_leftEncoderPrevDistance = getLeftPos()/ENCODER_COUNTS_PER_WHEEL_REV * Constants.DRIVE_WHEEL_DIAMETER_IN * Math.PI;
	        _rightEncoderPrevDistance = getLeftPos()/ENCODER_COUNTS_PER_WHEEL_REV * Constants.DRIVE_WHEEL_DIAMETER_IN * Math.PI;
            RobotState.getInstance().resetDistanceDriven();
            _pathFollower = new PathFollower(path, reversed, path.maxAccel, path.maxDecel, path.inertiaSteeringGain);
            _chassisState = ChassisState.FOLLOW_PATH;
            _currentPath = path;
		} 
		else 
		{
        	setLeftRightCommand(ControlMode.Velocity, 0.0, 0.0);
        }
    }

  public void initiateRobotState()
  {
    _pathFollower=null;
    _currentPath=null;
    _leftEncoderPrevDistance = getLeftPosInches();
    _rightEncoderPrevDistance = getRightPosInches();
    _robotState.reset(Timer.getFPGATimestamp(), new RigidTransform());
  }
  public void estimateRobotState( double timestamp)
  {
    final double left_distance = getLeftPosInches();
    final double right_distance = getRightPosInches();
    final Rotation gyro_angle = Rotation.fromDegrees(_navX.getYaw());
    final Twist odometry_velocity = _robotState.generateOdometryFromSensors(
        left_distance - _leftEncoderPrevDistance, right_distance - _rightEncoderPrevDistance, gyro_angle);
    final Twist predicted_velocity = Kinematics.forwardKinematics(getLeftVelocityInchesPerSec(), getRightVelocityInchesPerSec());
    _robotState.addObservations(timestamp, odometry_velocity, predicted_velocity);
    _leftEncoderPrevDistance = left_distance;
    _rightEncoderPrevDistance = right_distance;
  }

	public void updatePathFollower(double timestamp) 
	{
		RigidTransform _robotPose = RobotState.getInstance().getLatestFieldToVehicle().getValue();
		Twist command = _pathFollower.update(timestamp, _robotPose, RobotState.getInstance().getDistanceDriven(), RobotState.getInstance().getPredictedVelocity().dx);
		if (!_pathFollower.isFinished()) 
		{
			Kinematics.DriveVelocity setpoint = Kinematics.inverseKinematics(command);
			final double maxDesired = Math.max(Math.abs(setpoint.left), Math.abs(setpoint.right));
      double scale;
      if(maxDesired > Constants.DRIVE_VELOCITY_MAX_SETPOINT)
      {
        scale =Constants.DRIVE_VELOCITY_MAX_SETPOINT / maxDesired;
      } 
      else
      {
        scale=1;
      }
      setLeftRightCommand(ControlMode.Velocity, inchesPerSecToNU(setpoint.left * scale), inchesPerSecToNU(setpoint.right * scale));
      _centerTargetVelocity = command.dx;
			_leftTargetVelocity = setpoint.left;
			_rightTargetVelocity = setpoint.right;
		} 
		else 
		{
			setLeftRightCommand(ControlMode.Velocity, 0.0, 0.0);
		}
	}
	public synchronized boolean isDoneWithPath() 
	{
		if (_chassisState == ChassisState.FOLLOW_PATH && _pathFollower != null)
		{
			if (_pathFollower.isFinished())
			{
				System.out.println("Chassis Done With Path");
				return true;
			}
			else
			{
				return false;
			}
		}
		else 
		{
			return true;
		}
    }

    /** Path following e-stop */
	public synchronized void forceDoneWithPath() 
	{
        if (_chassisState == ChassisState.FOLLOW_PATH && _pathFollower != null)
            _pathFollower.forceFinish();
		else {}
		
           // System.out.println("Robot is not in path following mode");
	}
	public synchronized double getRemainingPathDistance() {
		if (_pathFollower != null) {
			return _pathFollower.remainingPathLength();
		} 
		return 0;
  }

  public synchronized void setTargetAngleAndTurnDirection(double targetAngle, boolean isTurnRight) {
		_targetAngle = targetAngle;
		_isTurnRight = isTurnRight;
		_chassisState = ChassisState.AUTO_TURN;
	} 
  
  public void moveToTargetAngle() {
		if((_navX.getYaw() >= 0 && _targetAngle >= 0 && _isTurnRight && _navX.getYaw() > _targetAngle) ||
			(_navX.getYaw() >= 0 && _targetAngle < 0 && _isTurnRight) ||
			(_navX.getYaw() < 0 && _targetAngle < 0 && _isTurnRight && Math.abs(_navX.getYaw()) < Math.abs(_targetAngle))) {
			_angleError = 360 - _navX.getYaw() + _targetAngle;
		}
		else if((_navX.getYaw() >= 0 && _targetAngle >= 0 && _isTurnRight && _navX.getYaw() < _targetAngle)||
				(_navX.getYaw() >= 0 && _targetAngle >= 0 && !_isTurnRight && _navX.getYaw() > _targetAngle)||
				(_navX.getYaw() >= 0 && _targetAngle < 0 && !_isTurnRight) ||
				(_navX.getYaw() < 0 && _targetAngle >= 0 && _isTurnRight) ||
				(_navX.getYaw() < 0 && _targetAngle < 0 && _isTurnRight && Math.abs(_navX.getYaw()) > Math.abs(_targetAngle)) ||
				(_navX.getYaw() < 0 && _targetAngle < 0 && !_isTurnRight && Math.abs(_navX.getYaw()) < Math.abs(_targetAngle))) {
			_angleError = _targetAngle - _navX.getYaw();
		}		
		else if((_navX.getYaw() >= 0 && _targetAngle >= 0 && !_isTurnRight && _navX.getYaw() < _targetAngle)||
				(_navX.getYaw() < 0 && _targetAngle < 0 && !_isTurnRight && Math.abs(_navX.getYaw()) > Math.abs(_targetAngle))||
				(_navX.getYaw() < 0 && _targetAngle >= 0 && !_isTurnRight)) {
			_angleError = _targetAngle - _navX.getYaw() - 360;
		}			
    if(_angleError>300)
    {
      _angleError-=360;
    }
    else if(_angleError<-300)
    {
      _angleError+=360;
    }
		double encoderError = ENCODER_CODES_PER_DEGREE * _angleError;		
		double leftDriveTargetPos = (getLeftPos() + encoderError);
		double rightDriveTargetPos = (getRightPos() - encoderError);
		setLeftRightCommand(ControlMode.MotionMagic, leftDriveTargetPos, rightDriveTargetPos);
  }
  public void updateAngleError()
  {
    if((_navX.getYaw() >= 0 && _targetAngle >= 0 && _isTurnRight && _navX.getYaw() > _targetAngle) ||
    (_navX.getYaw() >= 0 && _targetAngle < 0 && _isTurnRight) ||
    (_navX.getYaw() < 0 && _targetAngle < 0 && _isTurnRight && Math.abs(_navX.getYaw()) < Math.abs(_targetAngle))) {
    _angleError = 360 - _navX.getYaw() + _targetAngle;
  }
  else if((_navX.getYaw() >= 0 && _targetAngle >= 0 && _isTurnRight && _navX.getYaw() < _targetAngle)||
      (_navX.getYaw() >= 0 && _targetAngle >= 0 && !_isTurnRight && _navX.getYaw() > _targetAngle)||
      (_navX.getYaw() >= 0 && _targetAngle < 0 && !_isTurnRight) ||
      (_navX.getYaw() < 0 && _targetAngle >= 0 && _isTurnRight) ||
      (_navX.getYaw() < 0 && _targetAngle < 0 && _isTurnRight && Math.abs(_navX.getYaw()) > Math.abs(_targetAngle)) ||
      (_navX.getYaw() < 0 && _targetAngle < 0 && !_isTurnRight && Math.abs(_navX.getYaw()) < Math.abs(_targetAngle))) {
    _angleError = _targetAngle - _navX.getYaw();
  }		
  else if((_navX.getYaw() >= 0 && _targetAngle >= 0 && !_isTurnRight && _navX.getYaw() < _targetAngle)||
      (_navX.getYaw() < 0 && _targetAngle < 0 && !_isTurnRight && Math.abs(_navX.getYaw()) > Math.abs(_targetAngle))||
      (_navX.getYaw() < 0 && _targetAngle >= 0 && !_isTurnRight)) {
    _angleError = _targetAngle - _navX.getYaw() - 360;
  }			
  if(_angleError>300)
  {
    _angleError-=360;
  }
  else if(_angleError<-300)
  {
    _angleError+=360;
  }
  }
	//=====================================================================================
	// Helper Methods
  //=====================================================================================
  public void setLeftRightCommand(ControlMode mode, double leftCommand, double rightCommand) 
  {
		_leftMaster.set(mode, leftCommand);
		_rightMaster.set(mode, rightCommand);
  }
  public double getAngleError()
  {
    return _angleError;
  }
  public void setCanSeeTarget(boolean canSeeTarget)
  {
    _isVisionTargetVisible = canSeeTarget;
  }
  public boolean getIsVisionTargetVisible()
  {
    return _isVisionTargetVisible;
  }
  public double getDistanceToTargetInches()
  {
    return _limeLight.get_revisedDistance();
  }
  public void setBrakeMode(NeutralMode mode)
  {
    _leftMaster.setNeutralMode(mode);
		_leftSlave.setNeutralMode(mode);
		_rightMaster.setNeutralMode(mode);
    _rightSlave.setNeutralMode(mode);
  }
  public void setChassisState(ChassisState state)
  {
    _chassisState = state;
  }
  public double getHeading()
  {
    return _navX.getYaw();
  }

  public double getPositiveHeading(){
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
    _leftMaster.getSensorCollection().setQuadraturePosition(0, 10);
    _rightMaster.getSensorCollection().setQuadraturePosition(0,10);
    _navX.zeroYaw();
  }

  private static double rpmToInchesPerSecond(double rpm) 
	{
        return rotationsToInches(rpm) / 60;
    }
    
	private static double rotationsToInches(double rot) 
	{
        return rot * (Constants.DRIVE_WHEEL_DIAMETER_IN * Math.PI);
    } 

	private static double InchestoNU (double inches)
	{
		return inches * ENCODER_COUNTS_PER_WHEEL_REV/(Constants.DRIVE_WHEEL_DIAMETER_IN * Math.PI);
	}
	private static double NUtoInches (double NU)
	{
		return NU *Constants.DRIVE_WHEEL_DIAMETER_IN*Math.PI / ENCODER_COUNTS_PER_WHEEL_REV;
	}

	public static double inchesPerSecToNU(double inches_per_second) 
	{
        return inches_per_second * ENCODER_COUNTS_PER_WHEEL_REV / (Constants.DRIVE_WHEEL_DIAMETER_IN * Math.PI * 10);
	}

	public static double NUper100msToInchesPerSec(double NU_per_100ms)
	{
		return NU_per_100ms*10*Constants.DRIVE_WHEEL_DIAMETER_IN*Math.PI/(ENCODER_COUNTS_PER_WHEEL_REV);
	}
	public double getLeftSpeedRPM() 
	{
		return _leftMaster.getSelectedSensorVelocity(0) * (600 / ENCODER_COUNTS_PER_WHEEL_REV);
	}
	
	public double getRightSpeedRPM() 
	{
		return _rightMaster.getSelectedSensorVelocity(0) * (600 / ENCODER_COUNTS_PER_WHEEL_REV);
	}
	public double getLeftVelocityInchesPerSec() 
	{
        return rpmToInchesPerSecond(getLeftSpeedRPM());
    }

	public double getRightVelocityInchesPerSec() 
	{
        return rpmToInchesPerSecond(getRightSpeedRPM());
  }
  public double getLeftPosInches()
  {
    return getLeftPos()/ENCODER_COUNTS_PER_WHEEL_REV*6*Math.PI;
  }
  public double getRightPosInches()
  {
    return getRightPos()/ENCODER_COUNTS_PER_WHEEL_REV*6*Math.PI;
  }
  
  @Override
  public void updateLogData(LogDataBE logData) 
  {
    logData.AddData("Left Target Velocity", Double.toString(_leftTargetVelocity));
    logData.AddData("Left Actual Velocity", Double.toString(getLeftVelocityInchesPerSec()));

    logData.AddData("Right Target Velocity", Double.toString(_rightTargetVelocity));
    logData.AddData("Right Actual Velocity", Double.toString(getRightVelocityInchesPerSec()));

  }

  @Override
  public void updateDashboard() 
  {
    SmartDashboard.putNumber("Left Velo",getLeftVelocityInchesPerSec());
    SmartDashboard.putNumber("Right Velo", getRightVelocityInchesPerSec());
  }


}