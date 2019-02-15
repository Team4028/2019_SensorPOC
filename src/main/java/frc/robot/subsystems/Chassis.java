package frc.robot.subsystems;

import com.ctre.phoenix.motorcontrol.ControlMode;
import com.ctre.phoenix.motorcontrol.FeedbackDevice;
import com.ctre.phoenix.motorcontrol.LimitSwitchNormal;
import com.ctre.phoenix.motorcontrol.LimitSwitchSource;
import com.ctre.phoenix.motorcontrol.NeutralMode;
import com.ctre.phoenix.motorcontrol.StatusFrame;
import com.ctre.phoenix.motorcontrol.VelocityMeasPeriod;
import com.ctre.phoenix.motorcontrol.can.TalonSRX;

import edu.wpi.first.wpilibj.command.Subsystem;
import frc.robot.Constants;
import frc.robot.RobotMap;
import frc.robot.auton.pathfollowing.RobotState;
import frc.robot.auton.pathfollowing.control.Path;
import frc.robot.auton.pathfollowing.control.PathFollower;
import frc.robot.auton.pathfollowing.motion.RigidTransform;
import frc.robot.auton.pathfollowing.motion.Rotation;
import frc.robot.auton.pathfollowing.motion.Twist;
import frc.robot.auton.pathfollowing.util.Kinematics;
import frc.robot.interfaces.IBeakSquadSubsystem;
import frc.robot.sensors.GyroNavX;
import frc.robot.util.LogDataBE;

public class Chassis extends Subsystem implements IBeakSquadSubsystem {
  // =================================================================================================================
  // define class level working variables
  ChassisState _chassisState = ChassisState.UNKNOWN;
  GyroNavX _navX = GyroNavX.getInstance();
  private PathFollower _pathFollower;
  public TalonSRX _chassisLeftMasterMotor, _chassisLeftSlaveMotor, _chassisRightMasterMotor, _chassisRightSlaveMotor;

  public double _leftMtrDriveSetDistanceCmd, _rightMtrDriveSetDistanceCmd;

  public double _targetAngle,_angleError;

  private boolean _isTurnRight;

  private double _leftTargetVelocity, _rightTargetVelocity, _centerTargetVelocity;
  private Path _currentPath;
  private RobotState _robotState = RobotState.getInstance();
  private double _leftEncoderPrevDistance, _rightEncoderPrevDistance;

  public static double _autoStartTime;

  public static double ENCODER_COUNTS_PER_WHEEL_REV = 30028.471298; // 13582.78; 
  private double ENCODER_CODES_PER_DEGREE = 312; //value calculated theoretically from ECPWR //339.2468*100/169.8;

  // =================================================================================================================
  // Define Enums for the Chassis
  public enum ChassisState {
		UNKNOWN,
		PERCENT_VBUS,
		AUTO_TURN, 
		FOLLOW_PATH,
		DRIVE_SET_DISTANCE
  }

  // =================================================================================================================
  // define PID Constants
	private static final int CHASSIS_DEFAULT_PID_SLOT_INDEX = 0;
  
  private static final double FEED_FORWARD_GAIN_AUTO_TURN = 0.0581;
  private static final double PROPORTIONAL_GAIN_AUTO_TURN = 0.2;
  private static final double INTEGRAL_GAIN_AUTO_TURN = 0;
  private static final int INTEGRAL_ZONE_AUTO_TURN = 0;
  private static final double DERIVATIVE_GAIN_AUTO_TURN = 2;

  public static final double FEED_FORWARD_GAIN_DRIVE_SET_DISTANCE = 0.0357942617214836;
	public static final double PROPORTIONAL_GAIN_DRIVE_SET_DISTANCE = 0.175;
	public static final double INTEGRAL_GAIN_DRIVE_SET_DISTANCE = 0;
	public static final int INTEGRAL_ZONE_DRIVE_SET_DISTANCE = 0; 
	public static final double DERIVATIVE_GAIN_DRIVE_SET_DISTANCE = 1.75;
	
	public static final double FEED_FORWARD_GAIN_FOLLOW_PATH = 0.0357942617214836;
	public static final double PROPORTIONAL_GAIN_FOLLOW_PATH = 0.05;
	public static final double INTEGRAL_GAIN_FOLLOW_PATH = 0;
	public static final int INTEGRAL_ZONE_FOLLOW_PATH = 0; 
  public static final double DERIVATIVE_GAIN_FOLLOW_PATH = 0.85;
  
  private static final int CRUISE_VELOCITY_AUTO_TURN = 4000;
  private static final int CRUISE_ACCELERATION_AUTO_TURN = 30000;
  private static final int CRUISE_VELOCITY_DRIVE_SET_DISTANCE = 10000;
  private static final int CRUISE_ACCELERATION_DRIVE_SET_DISTANCE = 30000;
  //private static final int CRUISE_VELOCITY_FOLLOW_PATH = 1000;
  //private static final int CRUISE_ACCELERATION_FOLLOW_PATH = 4500;

	//=====================================================================================
	// Define Singleton Pattern
	//=====================================================================================
  private static Chassis _instance = new Chassis();
  
	public static Chassis getInstance() {
		return _instance;
	}
	
	// private constructor for singleton pattern
  private Chassis() {
    _chassisLeftMasterMotor = new TalonSRX(RobotMap.LEFT_DRIVE_MASTER_CAN_ADDR);
    _chassisLeftSlaveMotor = new TalonSRX(RobotMap.LEFT_DRIVE_SLAVE_CAN_ADDR);
    _chassisRightMasterMotor = new TalonSRX(RobotMap.RIGHT_DRIVE_MASTER_CAN_ADDR);
    _chassisRightSlaveMotor = new TalonSRX(RobotMap.RIGHT_DRIVE_SLAVE_CAN_ADDR);

    _chassisLeftSlaveMotor.follow(_chassisLeftMasterMotor);
    _chassisRightSlaveMotor.follow(_chassisRightMasterMotor);

    _chassisRightMasterMotor.setInverted(false);
    _chassisRightSlaveMotor.setInverted(false);
    _chassisLeftMasterMotor.setInverted(true);
    _chassisLeftSlaveMotor.setInverted(true);

    _chassisRightMasterMotor.setSensorPhase(false);
    _chassisLeftMasterMotor.setSensorPhase(false);

    _chassisLeftMasterMotor.setNeutralMode(NeutralMode.Brake);
		_chassisLeftSlaveMotor.setNeutralMode(NeutralMode.Brake);
		_chassisRightMasterMotor.setNeutralMode(NeutralMode.Brake);
    _chassisRightSlaveMotor.setNeutralMode(NeutralMode.Brake);
    
    configMasterMotors(_chassisLeftMasterMotor);
		configMasterMotors(_chassisRightMasterMotor);
    
    configDriveMotors(_chassisLeftMasterMotor);
    configDriveMotors(_chassisRightMasterMotor);
    configDriveMotors(_chassisLeftSlaveMotor);
		configDriveMotors(_chassisRightSlaveMotor);
  }

  private void configMasterMotors(TalonSRX talon) {
		talon.configSelectedFeedbackSensor(FeedbackDevice.CTRE_MagEncoder_Relative, 0, 10);
		talon.setStatusFramePeriod(StatusFrame.Status_2_Feedback0, 5, 10);
	
    talon.configVelocityMeasurementPeriod(VelocityMeasPeriod.Period_10Ms, 0);
    talon.configVelocityMeasurementWindow(32, 0);
    
    talon.configOpenloopRamp(0.4, 10);
		talon.configClosedloopRamp(0.0, 0);
	}
	
	private void configDriveMotors(TalonSRX talon) {
		talon.configForwardLimitSwitchSource(LimitSwitchSource.Deactivated, LimitSwitchNormal.Disabled, 0);
		talon.configReverseLimitSwitchSource(LimitSwitchSource.Deactivated, LimitSwitchNormal.Disabled, 0);
    
    talon.enableCurrentLimit(false);
    
    talon.configPeakOutputForward(1.0, 10);
    talon.configPeakOutputReverse(-1.0, 10);
    talon.configNominalOutputForward(0, 10);
    talon.configNominalOutputReverse(0, 10);
    talon.configContinuousCurrentLimit(Constants.BIG_NUMBER, 10);
  }
  
  public void updateChassis(double timestamp) {
    estimateRobotState(timestamp);
		switch(_chassisState) {
			case UNKNOWN:
        break;
      case PERCENT_VBUS:
				break;
      case AUTO_TURN:
        _chassisLeftMasterMotor.config_kF(CHASSIS_DEFAULT_PID_SLOT_INDEX, FEED_FORWARD_GAIN_AUTO_TURN);
        _chassisLeftMasterMotor.config_kP(CHASSIS_DEFAULT_PID_SLOT_INDEX, PROPORTIONAL_GAIN_AUTO_TURN);
        _chassisLeftMasterMotor.config_kI(CHASSIS_DEFAULT_PID_SLOT_INDEX, INTEGRAL_GAIN_AUTO_TURN);
        _chassisLeftMasterMotor.config_IntegralZone(CHASSIS_DEFAULT_PID_SLOT_INDEX, INTEGRAL_ZONE_AUTO_TURN);
        _chassisLeftMasterMotor.config_kD(CHASSIS_DEFAULT_PID_SLOT_INDEX, DERIVATIVE_GAIN_AUTO_TURN);
        _chassisRightMasterMotor.config_kF(CHASSIS_DEFAULT_PID_SLOT_INDEX, FEED_FORWARD_GAIN_AUTO_TURN);
        _chassisRightMasterMotor.config_kP(CHASSIS_DEFAULT_PID_SLOT_INDEX, PROPORTIONAL_GAIN_AUTO_TURN);
        _chassisRightMasterMotor.config_kI(CHASSIS_DEFAULT_PID_SLOT_INDEX, INTEGRAL_GAIN_AUTO_TURN);
        _chassisRightMasterMotor.config_IntegralZone(CHASSIS_DEFAULT_PID_SLOT_INDEX, INTEGRAL_ZONE_AUTO_TURN);
        _chassisRightMasterMotor.config_kD(CHASSIS_DEFAULT_PID_SLOT_INDEX, DERIVATIVE_GAIN_AUTO_TURN);
        _chassisRightMasterMotor.configMotionCruiseVelocity(CRUISE_VELOCITY_AUTO_TURN);
        _chassisLeftMasterMotor.configMotionCruiseVelocity(CRUISE_VELOCITY_AUTO_TURN);
        _chassisRightMasterMotor.configMotionAcceleration(CRUISE_ACCELERATION_AUTO_TURN);
        _chassisLeftMasterMotor.configMotionAcceleration(CRUISE_ACCELERATION_AUTO_TURN);
				break;
      case DRIVE_SET_DISTANCE:
        _chassisLeftMasterMotor.config_kF(CHASSIS_DEFAULT_PID_SLOT_INDEX, FEED_FORWARD_GAIN_DRIVE_SET_DISTANCE);
        _chassisLeftMasterMotor.config_kP(CHASSIS_DEFAULT_PID_SLOT_INDEX, PROPORTIONAL_GAIN_DRIVE_SET_DISTANCE);
        _chassisLeftMasterMotor.config_kI(CHASSIS_DEFAULT_PID_SLOT_INDEX, INTEGRAL_GAIN_DRIVE_SET_DISTANCE);
        _chassisLeftMasterMotor.config_IntegralZone(CHASSIS_DEFAULT_PID_SLOT_INDEX, INTEGRAL_ZONE_DRIVE_SET_DISTANCE);
        _chassisLeftMasterMotor.config_kD(CHASSIS_DEFAULT_PID_SLOT_INDEX, DERIVATIVE_GAIN_DRIVE_SET_DISTANCE);
        _chassisRightMasterMotor.config_kF(CHASSIS_DEFAULT_PID_SLOT_INDEX, FEED_FORWARD_GAIN_DRIVE_SET_DISTANCE);
        _chassisRightMasterMotor.config_kP(CHASSIS_DEFAULT_PID_SLOT_INDEX, PROPORTIONAL_GAIN_DRIVE_SET_DISTANCE);
        _chassisRightMasterMotor.config_kI(CHASSIS_DEFAULT_PID_SLOT_INDEX, INTEGRAL_GAIN_DRIVE_SET_DISTANCE);
        _chassisRightMasterMotor.config_IntegralZone(CHASSIS_DEFAULT_PID_SLOT_INDEX, INTEGRAL_ZONE_DRIVE_SET_DISTANCE);
        _chassisRightMasterMotor.config_kD(CHASSIS_DEFAULT_PID_SLOT_INDEX, DERIVATIVE_GAIN_DRIVE_SET_DISTANCE);
        _chassisRightMasterMotor.configMotionCruiseVelocity(CRUISE_VELOCITY_DRIVE_SET_DISTANCE);
        _chassisLeftMasterMotor.configMotionCruiseVelocity(CRUISE_VELOCITY_DRIVE_SET_DISTANCE);
        _chassisRightMasterMotor.configMotionAcceleration(CRUISE_ACCELERATION_DRIVE_SET_DISTANCE);
        _chassisLeftMasterMotor.configMotionAcceleration(CRUISE_ACCELERATION_DRIVE_SET_DISTANCE);
				break;
      case FOLLOW_PATH:
        estimateRobotState(timestamp);
        _chassisLeftMasterMotor.config_kF(CHASSIS_DEFAULT_PID_SLOT_INDEX, FEED_FORWARD_GAIN_FOLLOW_PATH);
        _chassisLeftMasterMotor.config_kP(CHASSIS_DEFAULT_PID_SLOT_INDEX, PROPORTIONAL_GAIN_FOLLOW_PATH);
        _chassisLeftMasterMotor.config_kI(CHASSIS_DEFAULT_PID_SLOT_INDEX, INTEGRAL_GAIN_FOLLOW_PATH);
        _chassisLeftMasterMotor.config_IntegralZone(CHASSIS_DEFAULT_PID_SLOT_INDEX, INTEGRAL_ZONE_FOLLOW_PATH);
        _chassisLeftMasterMotor.config_kD(CHASSIS_DEFAULT_PID_SLOT_INDEX, DERIVATIVE_GAIN_FOLLOW_PATH);
        _chassisRightMasterMotor.config_kF(CHASSIS_DEFAULT_PID_SLOT_INDEX, FEED_FORWARD_GAIN_FOLLOW_PATH);
        _chassisRightMasterMotor.config_kP(CHASSIS_DEFAULT_PID_SLOT_INDEX, PROPORTIONAL_GAIN_FOLLOW_PATH);
        _chassisRightMasterMotor.config_kI(CHASSIS_DEFAULT_PID_SLOT_INDEX, INTEGRAL_GAIN_FOLLOW_PATH);
        _chassisRightMasterMotor.config_IntegralZone(CHASSIS_DEFAULT_PID_SLOT_INDEX, INTEGRAL_ZONE_FOLLOW_PATH);
        _chassisRightMasterMotor.config_kD(CHASSIS_DEFAULT_PID_SLOT_INDEX, DERIVATIVE_GAIN_FOLLOW_PATH);
				if (_pathFollower != null) {
          updatePathFollower(timestamp);
        }
				break;
		}
  }
  
    // basic driving
    public void arcadeDrive(double throttleCmd, double turnCmd) {
      _chassisLeftMasterMotor.set(ControlMode.PercentOutput, throttleCmd+0.7*turnCmd);
      _chassisRightMasterMotor.set(ControlMode.PercentOutput, throttleCmd-0.7*turnCmd);
    }
  
    public void stop() {
      _chassisState = ChassisState.PERCENT_VBUS;
      setLeftRightCommand(ControlMode.PercentOutput, 0,0);
    }

  public synchronized void setTargetAngleAndTurnDirection(double targetAngle, boolean isTurnRight) {
    if(targetAngle>=0) {
      _targetAngle=targetAngle;
    } else {
      _targetAngle=360+targetAngle;
    }
		_isTurnRight = isTurnRight;
		_chassisState = ChassisState.AUTO_TURN;
  }
  
  public void moveToTargetAngle() {
    if((!_isTurnRight && getPositiveHeading() > _targetAngle) || (_isTurnRight && getPositiveHeading() < _targetAngle)) {
        _angleError = _targetAngle - getPositiveHeading();
    }           
    else if(!_isTurnRight && getPositiveHeading() < _targetAngle) {
        _angleError = _targetAngle - getPositiveHeading() - 360;
    }
    else if(_isTurnRight && getPositiveHeading() > _targetAngle) {
        _angleError = 360 - getPositiveHeading() + _targetAngle;
    }
    // System.out.println("AngleError:"+_angleError);
    double encoderError = ENCODER_CODES_PER_DEGREE * _angleError;       
    double leftDriveTargetPos = getLeftPos() + encoderError;
    double rightDriveTargetPos = getRightPos() - encoderError;
    
    setLeftRightCommand(ControlMode.MotionMagic, leftDriveTargetPos, rightDriveTargetPos);   
  }

  public void setMotionMagicCmdInches(double Distance) {

		_leftMtrDriveSetDistanceCmd = _chassisLeftMasterMotor.getSelectedSensorPosition(0)+ InchestoNU(Distance);
    _rightMtrDriveSetDistanceCmd = _chassisRightMasterMotor.getSelectedSensorPosition(0)+InchestoNU(Distance);
    _chassisState=ChassisState.DRIVE_SET_DISTANCE;
  }
  
  public void moveToTargetPosDriveSetDistance() {
		setLeftRightCommand(ControlMode.MotionMagic, _leftMtrDriveSetDistanceCmd, _rightMtrDriveSetDistanceCmd);
  }
  
  public synchronized void setWantDrivePath(Path path, boolean reversed) {
		if (_currentPath != path || _chassisState != ChassisState.FOLLOW_PATH) {
			_leftEncoderPrevDistance = getLeftPos()/ENCODER_COUNTS_PER_WHEEL_REV * Constants.DRIVE_WHEEL_DIAMETER_IN * Math.PI;
	        _rightEncoderPrevDistance = getLeftPos()/ENCODER_COUNTS_PER_WHEEL_REV * Constants.DRIVE_WHEEL_DIAMETER_IN * Math.PI;
            RobotState.getInstance().resetDistanceDriven();
            _pathFollower = new PathFollower(path, reversed, path.maxAccel, path.maxDecel, path.inertiaSteeringGain);
            _chassisState = ChassisState.FOLLOW_PATH;
            _currentPath = path;
		} else {
      setLeftRightCommand(ControlMode.Velocity, 0.0, 0.0);
    }
  }

  public synchronized void SetDynamicWantDrivePath(Path path, boolean reversed) {
    _leftEncoderPrevDistance = getLeftPos()/ENCODER_COUNTS_PER_WHEEL_REV * Constants.DRIVE_WHEEL_DIAMETER_IN * Math.PI;
	        _rightEncoderPrevDistance = getLeftPos()/ENCODER_COUNTS_PER_WHEEL_REV * Constants.DRIVE_WHEEL_DIAMETER_IN * Math.PI;
            RobotState.getInstance().resetDistanceDriven();
            _pathFollower = new PathFollower(path, reversed, path.maxAccel, path.maxDecel, path.inertiaSteeringGain);
            _chassisState = ChassisState.FOLLOW_PATH;
            _currentPath = path; 
  }

  public void estimateRobotState( double timestamp) {
    final double left_distance = NUtoInches(getLeftPos());
    final double right_distance = NUtoInches(getRightPos());
    final Rotation gyro_angle = Rotation.fromDegrees(_navX.getYaw());
    final Twist odometry_velocity = _robotState.generateOdometryFromSensors(
        left_distance - _leftEncoderPrevDistance, right_distance - _rightEncoderPrevDistance, gyro_angle);
    final Twist predicted_velocity = Kinematics.forwardKinematics(getLeftVelocityInchesPerSec(),
        getRightVelocityInchesPerSec());
    _robotState.addObservations(timestamp, odometry_velocity, predicted_velocity);
    _leftEncoderPrevDistance = left_distance;
    _rightEncoderPrevDistance = right_distance;
  }

	public void updatePathFollower(double timestamp) 
	{
		RigidTransform _robotPose = RobotState.getInstance().getLatestFieldToVehicle().getValue();
		Twist command = _pathFollower.update(timestamp, _robotPose, RobotState.getInstance().getDistanceDriven(), RobotState.getInstance().getPredictedVelocity().dx);
		if (!_pathFollower.isFinished()) {
			Kinematics.DriveVelocity setpoint = Kinematics.inverseKinematics(command);
			final double maxDesired = Math.max(Math.abs(setpoint.left), Math.abs(setpoint.right));
      double scale;
      if(maxDesired > Constants.DRIVE_VELOCITY_MAX_SETPOINT) {
        scale =Constants.DRIVE_VELOCITY_MAX_SETPOINT / maxDesired;
      } else {
        scale=1;
      }
      setLeftRightCommand(ControlMode.Velocity, inchesPerSecToNU(setpoint.left * scale), inchesPerSecToNU(setpoint.right * scale));
      _centerTargetVelocity = command.dx;
			_leftTargetVelocity = setpoint.left;
			_rightTargetVelocity = setpoint.right;
		} else {
			setLeftRightCommand(ControlMode.Velocity, 0.0, 0.0);
		}
  }
  
	public synchronized boolean isDoneWithPath() {
		if (_chassisState == ChassisState.FOLLOW_PATH && _pathFollower != null) {
			if (_pathFollower.isFinished()) {
				System.out.println("Chassis Done With Path");
				return true;
			}	else {
				return false;
			}
		}	else {
			return true;
		}
  }

  /** Path following e-stop */
	public synchronized void forceDoneWithPath() {
    if (_chassisState == ChassisState.FOLLOW_PATH && _pathFollower != null) {
      _pathFollower.forceFinish();
    } else {}
  }
  
	public synchronized double getRemainingPathDistance() {
		if (_pathFollower != null) {
			return _pathFollower.remainingPathLength();
		} 
		return 0;
	}
	//=====================================================================================
	// Helper Methods
  //=====================================================================================
  public void setLeftRightCommand(ControlMode mode, double leftCommand, double rightCommand) {
		_chassisLeftMasterMotor.set(mode, leftCommand);
		_chassisRightMasterMotor.set(mode, rightCommand);
  }
  
  public double getHeading() {
    return _navX.getYaw();
  }

  public double getPositiveHeading() {
    if(_navX.getYaw()<0) {
      return 360+_navX.getYaw();
    } else {
      return _navX.getYaw();
    }
  }

  public double getLeftPos() {
		return _chassisLeftMasterMotor.getSelectedSensorPosition(0);
  }
  
	public double getRightPos() {
		return _chassisRightMasterMotor.getSelectedSensorPosition(0);
  }

  public void zeroSensors() {
    _chassisLeftMasterMotor.setSelectedSensorPosition(0);
    _chassisRightMasterMotor.setSelectedSensorPosition(0);
    _navX.zeroYaw();
  }
  // public void setOffset(double os){
  //   _navX.setOffset(os);
  // }
  // public void setGyroToZero(){
  //   _navX.reAxizeNow();
  // }
  private static double rpmToInchesPerSecond(double rpm) {
    return rotationsToInches(rpm) / 60;
  }
    
	private static double rotationsToInches(double rot) {
    return rot * (Constants.DRIVE_WHEEL_DIAMETER_IN * Math.PI);
  } 

	private static double InchestoNU (double inches) {
		return inches * ENCODER_COUNTS_PER_WHEEL_REV/(Constants.DRIVE_WHEEL_DIAMETER_IN * Math.PI);
  }
  
	private static double NUtoInches (double NU) {
		return NU *Constants.DRIVE_WHEEL_DIAMETER_IN*Math.PI / ENCODER_COUNTS_PER_WHEEL_REV;
	}

  // public double getCurOffset(){
  //   return _navX.getOffset();
  // }
	public static double inchesPerSecToNU(double inches_per_second) {
        return inches_per_second * ENCODER_COUNTS_PER_WHEEL_REV / (Constants.DRIVE_WHEEL_DIAMETER_IN * Math.PI * 10);
	}

	public static double NUper100msToInchesPerSec(double NU_per_100ms) {
		return NU_per_100ms*10*Constants.DRIVE_WHEEL_DIAMETER_IN*Math.PI/(ENCODER_COUNTS_PER_WHEEL_REV);
  }
  
	public double getLeftSpeedRPM() {
		return _chassisLeftMasterMotor.getSelectedSensorVelocity(0) * (600 / ENCODER_COUNTS_PER_WHEEL_REV);
	}
	
	public double getRightSpeedRPM() {
		return _chassisRightMasterMotor.getSelectedSensorVelocity(0) * (600 / ENCODER_COUNTS_PER_WHEEL_REV);
  }
  
	public double getLeftVelocityInchesPerSec() {
    return rpmToInchesPerSecond(getLeftSpeedRPM());
  }

	public double getRightVelocityInchesPerSec() {
    return rpmToInchesPerSecond(getRightSpeedRPM());
	}
  
  // ===============================================================================================================
	// Default Command
	// ===============================================================================================================
  @Override
  public void initDefaultCommand() {}

  // ===============================================================================================================
	// General Purpose Utility Methods
	// ===============================================================================================================
  @Override
  public void updateLogData(LogDataBE logData) {
    logData.AddData("Left Target Velocity", Double.toString(_leftTargetVelocity));
    logData.AddData("Left Actual Velocity", Double.toString(getLeftVelocityInchesPerSec()));

    logData.AddData("Right Target Velocity", Double.toString(_rightTargetVelocity));
    logData.AddData("Right Actual Velocity", Double.toString(getRightVelocityInchesPerSec()));
  }

  @Override
  public void updateDashboard() {}
}