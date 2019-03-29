package frc.robot.subsystems;

//#region
import com.ctre.phoenix.motorcontrol.ControlMode;
import com.ctre.phoenix.motorcontrol.FeedbackDevice;
import com.ctre.phoenix.motorcontrol.LimitSwitchNormal;
import com.ctre.phoenix.motorcontrol.LimitSwitchSource;
import com.ctre.phoenix.motorcontrol.NeutralMode;
import com.ctre.phoenix.motorcontrol.StatusFrame;
import com.ctre.phoenix.motorcontrol.VelocityMeasPeriod;
import com.ctre.phoenix.motorcontrol.can.TalonSRX;

import edu.wpi.first.wpilibj.command.Subsystem;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import frc.robot.RobotMap;
import frc.robot.interfaces.IBeakSquadSubsystem;
import frc.robot.util.LogDataBE;
//#endregion

public class Elevator extends Subsystem implements IBeakSquadSubsystem {

  // =================================================================================================================
  // define class level working variables
  private static TalonSRX _elevatorMasterMotor;
  private static TalonSRX _elevatorSlaveMotor;

  private int _targetElevatorPositionNU;
  private boolean _hasElevatorBeenZeroed = false;

  private String _presetPositionName = "Unknown";

  // =================================================================================================================
  // Define Enums for the Elevator Axis
  public enum ELEVATOR_TARGET_POSITION {
    HOME,
    LEVEL_1,
    LEVEL_2,
    LEVEL_3,
    CARGO_ACQUIRE,
    NULL
  }

  // =================================================================================================================
  // Define Class Constants
  // =================================================================================================================
  //#region
  private static final int ELEVATOR_POS_ALLOWABLE_ERROR_NU = InchesToNativeUnits(0.5);

  //Conversion Constant
  private static final double NATIVE_UNITS_TO_INCHES_CONVERSION = 242.7928;

	// hardcoded preset positions (in native units, 0 = home position)
  private static final int HOME_POSITION_NU = InchesToNativeUnits(0);
  private static final int CARGO_LEVEL_1_POSITION_NU = InchesToNativeUnits(24.25);
  private static final int CARGO_LEVEL_2_POSITION_NU = InchesToNativeUnits(52.25);
  private static final int CARGO_LEVEL_3_POSITION_NU = InchesToNativeUnits(79.25);//52.25
  private static final int HATCH_LEVEL_1_POSITION_NU = InchesToNativeUnits(0);
  private static final int HATCH_LEVEL_2_POSITION_NU = InchesToNativeUnits(31);
  private static final int HATCH_LEVEL_3_POSITION_NU = InchesToNativeUnits(58);
  private static final int CARGO_ACQUIRE_HEIGHT = InchesToNativeUnits(38);

  ELEVATOR_TARGET_POSITION _storedPresetPosition;

  // define PID Constants
	private static final int MOVING_DOWN_PID_SLOT_INDEX = 0;
	private static final int MOVING_UP_PID_SLOT_INDEX = 1;
  private static final int HOLDING_PID_SLOT_INDEX = 2;
  
  private static final double FEED_FORWARD_GAIN_UP = 0.17427598;
  private static final double PROPORTIONAL_GAIN_UP = 2.0;
  private static final double INTEGRAL_GAIN_UP = 0;
  private static final int INTEGRAL_ZONE_UP = 0;
  private static final double DERIVATIVE_GAIN_UP = 30;

  public static final double FEED_FORWARD_GAIN_HOLD = 0.17427598;
	public static final double PROPORTIONAL_GAIN_HOLD = 2.0;
	public static final double INTEGRAL_GAIN_HOLD = 0.04;
	public static final int INTEGRAL_ZONE_HOLD = 200; 
	public static final double DERIVATIVE_GAIN_HOLD = 0;
	
	public static final double FEED_FORWARD_GAIN_DOWN = 0.17427598;
	public static final double PROPORTIONAL_GAIN_DOWN = 0.2;
	public static final double INTEGRAL_GAIN_DOWN = 0;
	public static final int INTEGRAL_ZONE_DOWN = 0; 
  public static final double DERIVATIVE_GAIN_DOWN = 0;
  
  private static final int UP_CRUISE_VELOCITY = 2500;
  private static final int DOWN_CRUISE_VELOCITY = 2000;
  private static final int TELEOP_UP_ACCELERATION = 4000;
  private static final int TELEOP_UP_DECELERATION = 4000;
  private static final int TELEOP_DOWN_ACCELERATION = 1000;

  private static final int CAN_TIMEOUT_MSECS_INIT = 10;
  private static final int CAN_TIMEOUT_MSECS_PERIODIC = 0;
  //#endregion

  //=====================================================================================
	// Define Singleton Pattern
	//=====================================================================================
  private static Elevator _instance = new Elevator();
  
  public static Elevator getInstance() {
    return _instance;
  }

  private Elevator() {
    // define master motor
    _elevatorMasterMotor = new TalonSRX(RobotMap.ELEVATOR_MASTER_CAN_ADDR);
    _elevatorMasterMotor.configFactoryDefault();

    // define slave motor
    _elevatorSlaveMotor = new TalonSRX(RobotMap.ELEVATOR_SLAVE_CAN_ADDR);
    _elevatorSlaveMotor.configFactoryDefault();
    _elevatorSlaveMotor.follow(_elevatorMasterMotor);

    // Set motor phasing
    _elevatorMasterMotor.setInverted(false);

    // Configure Limit Switch
    _elevatorMasterMotor.configForwardLimitSwitchSource(LimitSwitchSource.FeedbackConnector,
        LimitSwitchNormal.NormallyOpen);
    _elevatorMasterMotor.configReverseLimitSwitchSource(LimitSwitchSource.FeedbackConnector,
        LimitSwitchNormal.NormallyOpen);

    // Turn of all soft limits
    _elevatorMasterMotor.configForwardSoftLimitEnable(false, CAN_TIMEOUT_MSECS_INIT);
    _elevatorMasterMotor.configReverseSoftLimitEnable(false, CAN_TIMEOUT_MSECS_INIT);

    // Configure brake mode
    _elevatorMasterMotor.setNeutralMode(NeutralMode.Brake);

    // Configure Quad Encoder (Invert = false)
    _elevatorMasterMotor.configSelectedFeedbackSensor(FeedbackDevice.CTRE_MagEncoder_Relative, 0, CAN_TIMEOUT_MSECS_INIT);
    _elevatorMasterMotor.setSensorPhase(false);
    _elevatorMasterMotor.setStatusFramePeriod(StatusFrame.Status_2_Feedback0, 5, CAN_TIMEOUT_MSECS_INIT);

    // Peak/Nominal output voltages for both directions for talons configuration
    _elevatorMasterMotor.configNominalOutputForward(0, CAN_TIMEOUT_MSECS_INIT);
    _elevatorMasterMotor.configNominalOutputReverse(0, CAN_TIMEOUT_MSECS_INIT);
    _elevatorMasterMotor.configPeakOutputForward(1, CAN_TIMEOUT_MSECS_INIT);
    _elevatorMasterMotor.configPeakOutputReverse(-1, CAN_TIMEOUT_MSECS_INIT);

    // Configure velocity measurement
    _elevatorMasterMotor.configVelocityMeasurementPeriod(VelocityMeasPeriod.Period_5Ms, CAN_TIMEOUT_MSECS_INIT);
    _elevatorMasterMotor.configVelocityMeasurementWindow(32, CAN_TIMEOUT_MSECS_INIT);

    // Set up MotionMagic mode
    _elevatorMasterMotor.selectProfileSlot(MOVING_UP_PID_SLOT_INDEX, 0);

    // Set closed loop gains
    _elevatorMasterMotor.config_kF(MOVING_DOWN_PID_SLOT_INDEX, FEED_FORWARD_GAIN_DOWN, CAN_TIMEOUT_MSECS_INIT);
		_elevatorMasterMotor.config_kP(MOVING_DOWN_PID_SLOT_INDEX, PROPORTIONAL_GAIN_DOWN, CAN_TIMEOUT_MSECS_INIT);
		_elevatorMasterMotor.config_kI(MOVING_DOWN_PID_SLOT_INDEX, INTEGRAL_GAIN_DOWN, CAN_TIMEOUT_MSECS_INIT);
		_elevatorMasterMotor.config_kD(MOVING_DOWN_PID_SLOT_INDEX, DERIVATIVE_GAIN_DOWN, CAN_TIMEOUT_MSECS_INIT);
		_elevatorMasterMotor.config_IntegralZone(MOVING_DOWN_PID_SLOT_INDEX, INTEGRAL_ZONE_DOWN, CAN_TIMEOUT_MSECS_INIT);
		
		_elevatorMasterMotor.config_kF(MOVING_UP_PID_SLOT_INDEX, FEED_FORWARD_GAIN_UP, CAN_TIMEOUT_MSECS_INIT);
		_elevatorMasterMotor.config_kP(MOVING_UP_PID_SLOT_INDEX, PROPORTIONAL_GAIN_UP, CAN_TIMEOUT_MSECS_INIT);
		_elevatorMasterMotor.config_kI(MOVING_UP_PID_SLOT_INDEX, INTEGRAL_GAIN_UP, CAN_TIMEOUT_MSECS_INIT);
		_elevatorMasterMotor.config_kD(MOVING_UP_PID_SLOT_INDEX, DERIVATIVE_GAIN_UP, CAN_TIMEOUT_MSECS_INIT);
		_elevatorMasterMotor.config_IntegralZone(MOVING_UP_PID_SLOT_INDEX, INTEGRAL_ZONE_UP, CAN_TIMEOUT_MSECS_INIT);
		
		_elevatorMasterMotor.config_kF(HOLDING_PID_SLOT_INDEX, FEED_FORWARD_GAIN_HOLD, CAN_TIMEOUT_MSECS_INIT);
		_elevatorMasterMotor.config_kP(HOLDING_PID_SLOT_INDEX, PROPORTIONAL_GAIN_HOLD, CAN_TIMEOUT_MSECS_INIT);
		_elevatorMasterMotor.config_kI(HOLDING_PID_SLOT_INDEX, INTEGRAL_GAIN_HOLD, CAN_TIMEOUT_MSECS_INIT);
		_elevatorMasterMotor.config_kD(HOLDING_PID_SLOT_INDEX, DERIVATIVE_GAIN_HOLD, CAN_TIMEOUT_MSECS_INIT);
		_elevatorMasterMotor.config_IntegralZone(HOLDING_PID_SLOT_INDEX, INTEGRAL_ZONE_HOLD, CAN_TIMEOUT_MSECS_INIT);

    // Set accel and cruise velocities
    _elevatorMasterMotor.configMotionCruiseVelocity(UP_CRUISE_VELOCITY, CAN_TIMEOUT_MSECS_INIT);
    _elevatorMasterMotor.configMotionAcceleration(TELEOP_UP_ACCELERATION, CAN_TIMEOUT_MSECS_INIT);
  
    // set allowable closed loop gain
    _elevatorMasterMotor.configAllowableClosedloopError(0, ELEVATOR_POS_ALLOWABLE_ERROR_NU, CAN_TIMEOUT_MSECS_INIT);
  }

  // =================================================================================================================
	// Methods to Move the Elevator
	// =================================================================================================================
	
  public void zeroElevatorMotorEncoder() {
    if (_elevatorMasterMotor.getSensorCollection().isRevLimitSwitchClosed()) {
      _elevatorMasterMotor.setSelectedSensorPosition(0);
      _hasElevatorBeenZeroed = true;
    }
  }

  public void setTargetPosition(ELEVATOR_TARGET_POSITION presetPosition, boolean hasHatch) {
    switch(presetPosition){
      case HOME:
        _targetElevatorPositionNU = HOME_POSITION_NU;
        _storedPresetPosition= ELEVATOR_TARGET_POSITION.HOME;
        break;

      case LEVEL_1:
        _storedPresetPosition = ELEVATOR_TARGET_POSITION.LEVEL_1;
        if(hasHatch){
          _targetElevatorPositionNU = HATCH_LEVEL_1_POSITION_NU;
        } else {
          _targetElevatorPositionNU = CARGO_LEVEL_1_POSITION_NU;
        } 
        break;

      case LEVEL_2:
        _storedPresetPosition = ELEVATOR_TARGET_POSITION.LEVEL_2;
        if(hasHatch){
          _targetElevatorPositionNU = HATCH_LEVEL_2_POSITION_NU;
        } else {
          _targetElevatorPositionNU = CARGO_LEVEL_2_POSITION_NU;
        } 
        break;

      case LEVEL_3:
        _storedPresetPosition = ELEVATOR_TARGET_POSITION.LEVEL_3;
        if(hasHatch){
          _targetElevatorPositionNU = HATCH_LEVEL_3_POSITION_NU;
        } else {
          _targetElevatorPositionNU = CARGO_LEVEL_3_POSITION_NU;
        } 
        break;  

      case CARGO_ACQUIRE:
        _targetElevatorPositionNU = CARGO_ACQUIRE_HEIGHT;
        break;
    }
    _presetPositionName = presetPosition.toString();
  }

  public void moveToPresetPosition(){
    // set appropriate gain slot to use (only flip if outside deadband)
    int currentError = Math.abs(get_ElevatorPos() - _targetElevatorPositionNU);
    if (currentError > ELEVATOR_POS_ALLOWABLE_ERROR_NU) 
    {
      if(_targetElevatorPositionNU > get_ElevatorPos()) 
      {
        _elevatorMasterMotor.selectProfileSlot(MOVING_UP_PID_SLOT_INDEX, 0);
        _elevatorMasterMotor.configMotionCruiseVelocity(UP_CRUISE_VELOCITY, CAN_TIMEOUT_MSECS_PERIODIC);
        _elevatorMasterMotor.configMotionAcceleration(TELEOP_UP_ACCELERATION, CAN_TIMEOUT_MSECS_PERIODIC);
      } 
      else 
      {
        _elevatorMasterMotor.selectProfileSlot(MOVING_DOWN_PID_SLOT_INDEX, 0);
        _elevatorMasterMotor.configMotionCruiseVelocity(DOWN_CRUISE_VELOCITY, CAN_TIMEOUT_MSECS_PERIODIC);
        if(get_ElevatorVelocity() > 0)
        {
          _elevatorMasterMotor.configMotionAcceleration(TELEOP_UP_DECELERATION, CAN_TIMEOUT_MSECS_PERIODIC);
        } 
        else 
        {
          _elevatorMasterMotor.configMotionAcceleration(TELEOP_DOWN_ACCELERATION, CAN_TIMEOUT_MSECS_PERIODIC);
        }
      }
    } else {
      // _elevatorMasterMotor.selectProfileSlot(HOLDING_PID_SLOT_INDEX, 0);
    }
    _elevatorMasterMotor.set(ControlMode.MotionMagic, _targetElevatorPositionNU);
  }

  // ===============================================================================================================
	// Expose Properties of Elevator
	// ===============================================================================================================
  public boolean get_isElevatorAtTargetPos(){
    return get_isElevatorAtTargetPos(_targetElevatorPositionNU);
  }

  private boolean get_isElevatorAtTargetPos(int targetPosition) {
    int currentError = Math.abs(_elevatorMasterMotor.getSelectedSensorPosition() - targetPosition);
    if(currentError <= ELEVATOR_POS_ALLOWABLE_ERROR_NU) {
      return true;
    } else {
      return false;
    }
  }

  public ELEVATOR_TARGET_POSITION getStoredTargetPosition()
  {
    return _storedPresetPosition;
  }

  public boolean get_hasElevatorBeenZeroed() {
    return _hasElevatorBeenZeroed;
  }

  public int get_ElevatorPos() {
    return _elevatorMasterMotor.getSelectedSensorPosition(0);
  }

  private int get_ElevatorVelocity() {
    return _elevatorMasterMotor.getSelectedSensorVelocity();
  }

  // ===============================================================================================================
	// Private Helper Methods
	// ===============================================================================================================
  private double NativeUnitsToInches(double nativeUnitsMeasure) {
    double inches = nativeUnitsMeasure / NATIVE_UNITS_TO_INCHES_CONVERSION;
    return inches;
  }

  private static int InchesToNativeUnits(double inchesMeasure) {
    int nativeUnits = (int)(inchesMeasure * NATIVE_UNITS_TO_INCHES_CONVERSION);
    return nativeUnits;
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
  public void updateLogData(LogDataBE logData) {}

  @Override
  public void updateDashboard() {
    //SmartDashboard.putNumber("elevator pos", get_ElevatorPos());
    //SmartDashboard.putNumber("elevator:inches", NativeUnitsToInches(get_ElevatorPos()));
    //SmartDashboard.putNumber("elevator:native units", get_ElevatorPos());
    //SmartDashboard.putNumber("Elevator:masterMotorOutputVolts", _elevatorMasterMotor.getMotorOutputVoltage());
    //SmartDashboard.putNumber("Elevator:masterMotorCurrentAmps", _elevatorMasterMotor.getOutputCurrent());
    //SmartDashboard.putNumber("Elevator:slaveMotorOutputVolts", _elevatorSlaveMotor.getMotorOutputVoltage());
    //SmartDashboard.putNumber("Elevator:slaveMotorCurrentAmps", _elevatorSlaveMotor.getOutputCurrent());
    SmartDashboard.putNumber("Elevator: Target Position", NativeUnitsToInches(_targetElevatorPositionNU));
    SmartDashboard.putString("Elevator: Position", _presetPositionName);
  }
}
