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

import edu.wpi.first.wpilibj.DigitalInput;
import edu.wpi.first.wpilibj.command.Subsystem;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import frc.robot.RobotMap;
import frc.robot.util.GeneralUtilities;
import frc.robot.util.LogDataBE;
 

/**
 * Add your docs here.
 */

public class Elevator extends Subsystem {

  public enum ELEVATOR_TARGET_POSITION{
    HOME,
    HATCH_LEVEL_1,
    HATCH_LEVEL_2,
    HATCH_LEVEL_3,
    CARGO_LEVEL_1,
    CARGO_LEVEL_2,
    CARGO_LEVEL_3
  }
  public enum ELEVATOR_UP_OR_DOWN{
    UP,
    DOWN
  }

  private static TalonSRX _elevatorMasterMotor;
  private static TalonSRX _elevatorSlaveMotor;
  private double INCHES_TO_NATIVE_UNITS_CONVERSION = 242.7928;
  private double NATIVE_UNITS_TO_INCHES_CONVERSION = 0.004119;
  private static Elevator _instance = new Elevator();

  private static final double FEED_FORWARD_GAIN = 0.4;
  private static final double PROPORTIONAL_GAIN = 0;
  private static final double INTEGRAL_GAIN = 0;
  private static final int INTEGRAL_ZONE = 0;
  private static final double DERIVATIVE_GAIN = 0;
  private static final int CRUISE_VELOCITY = 4000;
  private static final int CRUISE_ACCELERATION = 4500;
  private static final int CAN_TIMEOUT_MILLISECONDS = 30;
  private double INCHES_TO_NATIVE_UNITS_CONVERSION = 242.7928;
  private double NATIVE_UNITS_TO_INCHES_CONVERSION = 0.004119;

  public static Elevator getInstance(){
    return _instance;
  }

  private Elevator(){
    // define master motor
    _elevatorMasterMotor = new TalonSRX(RobotMap.ELEVATOR_MASTER_CAN_ADDR);
    _elevatorMasterMotor.configFactoryDefault();

    // define slave motor
    _elevatorSlaveMotor = new TalonSRX(RobotMap.ELEVATOR_SLAVE_CAN_ADDR);
    _elevatorSlaveMotor.configFactoryDefault();
    _elevatorSlaveMotor.follow(_elevatorMasterMotor);

    // Set motor phasing
    _elevatorMasterMotor.setInverted(false);
    _elevatorSlaveMotor.setInverted(false);

    // Configure Limit Switch
    _elevatorMasterMotor.configForwardLimitSwitchSource(LimitSwitchSource.FeedbackConnector, LimitSwitchNormal.NormallyOpen);
    _elevatorMasterMotor.configReverseLimitSwitchSource(LimitSwitchSource.FeedbackConnector, LimitSwitchNormal.NormallyOpen);

    // Turn of all soft limits
    _elevatorMasterMotor.configForwardSoftLimitEnable(false, 0);
    _elevatorMasterMotor.configReverseSoftLimitEnable(false, 0);

    //Configure brake mode
    _elevatorMasterMotor.setNeutralMode(NeutralMode.Brake);
    _elevatorSlaveMotor.setNeutralMode(NeutralMode.Brake);

    // Configure Encoder
    _elevatorMasterMotor.configSelectedFeedbackSensor(FeedbackDevice.CTRE_MagEncoder_Relative, 0, 0);
    _elevatorMasterMotor.setSensorPhase(false);
    _elevatorMasterMotor.setStatusFramePeriod(StatusFrame.Status_2_Feedback0, 5, 0);
    
    // peak/nominal output voltages for both directions for talons configuration
    _elevatorMasterMotor.configNominalOutputForward(0, 0);
    _elevatorMasterMotor.configNominalOutputReverse(0, 0);
    _elevatorMasterMotor.configPeakOutputForward(1, 0);
    _elevatorMasterMotor.configPeakOutputReverse(-1, 0);
    
    //Configur velocity measurement
    _elevatorMasterMotor.configVelocityMeasurementPeriod(VelocityMeasPeriod.Period_5Ms, 0);
    _elevatorMasterMotor.configVelocityMeasurementWindow(32, 0);

    _elevatorMasterMotor.setSelectedSensorPosition(0, 0, 0);
    //Set up MotionMagic mode
    //SetPidSlotToUse("constr", MOVING_DOWN_PID_SLOT_INDEX)
    _elevatorMasterMotor.selectProfileSlot(0, 0);

    // Set closed loop gains
    _elevatorMasterMotor.config_kF(0, FEED_FORWARD_GAIN, CAN_TIMEOUT_MILLISECONDS);
    _elevatorMasterMotor.config_kP(0, PROPORTIONAL_GAIN, CAN_TIMEOUT_MILLISECONDS);
    _elevatorMasterMotor.config_kI(0, INTEGRAL_GAIN, CAN_TIMEOUT_MILLISECONDS);
    _elevatorMasterMotor.config_kD(0, DERIVATIVE_GAIN, CAN_TIMEOUT_MILLISECONDS);

    //Set accel and cruise velocities
    _elevatorMasterMotor.configMotionCruiseVelocity(CRUISE_VELOCITY, 0);
    _elevatorMasterMotor.configMotionAcceleration(CRUISE_ACCELERATION, 0);
    
  }

  public void moveElevator(ELEVATOR_UP_OR_DOWN elevatorUpOrDown){
    switch(elevatorUpOrDown){
      case UP:
          _elevatorMasterMotor.set(ControlMode.PercentOutput, .3);
        break;
      case DOWN:
        _elevatorMasterMotor.set(ControlMode.PercentOutput, -.1);
        break;
    }
  }

<<<<<<< HEAD
  public int get_ElevatorPos(){
    return _elevatorMasterMotor.getSelectedSensorPosition(0);
  }

  public double nativeUnitsToInches(double nativeUnitsMeasure) {
    double inches = nativeUnitsMeasure/ INCHES_TO_NATIVE_UNITS_CONVERSION;
    inches = GeneralUtilities.roundDouble(inches, 2);
    return inches;
  }

  public double InchesToNativeUints (double inchesMeasure) {
    double nativeUnits = inchesMeasure / NATIVE_UNITS_TO_INCHES_CONVERSION;
    return nativeUnits;
  }

  public void updateDashboard(){
    SmartDashboard.putNumber("Elevator:inches", nativeUnitsToInches(get_ElevatorPos()));
    SmartDashboard.putNumber("Elevator:nativeUnits", get_ElevatorPos());
  }
=======
  public void MoveElevatorToSelectedPosition(){
    _elevatorMasterMotor.set(ControlMode.MotionMagic, InchesToNativeUnits(12));
  }

  public void zeroElevatorMotorEncoder(){
    if(isBottomElevatorLimitSwitchClosed()){
      _elevatorMasterMotor.setSelectedSensorPosition(0);
    }
  }

  public int get_ElevatorPos(){
    return _elevatorMasterMotor.getSelectedSensorPosition(0);
  }
  public boolean isBottomElevatorLimitSwitchClosed(){
    return _elevatorMasterMotor.getSensorCollection().isRevLimitSwitchClosed();
  }

  public double NativeUnitsToInches(double nativeUnitsMeasure){
    double nativeUnits = nativeUnitsMeasure / INCHES_TO_NATIVE_UNITS_CONVERSION;
    return nativeUnits;
  }

  public double InchesToNativeUnits(double inchesMeasure){
    double inches = inchesMeasure / NATIVE_UNITS_TO_INCHES_CONVERSION;
    return inches;
  }

>>>>>>> 226ce4e7cf93a4cea37c75025a5c6e6a1abde339

  public void updateLogData(LogDataBE logData) {
  }
  
  @Override
  public void initDefaultCommand() {
    // Set the default command for a subsystem here.
    // setDefaultCommand(new MySpecialCommand());
  }
}
