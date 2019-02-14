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
import frc.robot.interfaces.IBeakSquadSubsystem;
import frc.robot.util.LogDataBE;

/**
 * Add your docs here.
 */

public class Elevator extends Subsystem implements IBeakSquadSubsystem {

   double _targetElevatorPositionNU;
   boolean _hasElevatorBeenZeroed = false;
  public enum ELEVATOR_TARGET_POSITION {
    HOME,
    CARGO_LEVEL_1,
    CARGO_LEVEL_2,
    CARGO_LEVEL_3,
    HATCH_LEVEL_1,
    HATCH_LEVEL_2,
    HATCH_LEVEL_3
  }

  private static TalonSRX _elevatorMasterMotor;
  private static TalonSRX _elevatorSlaveMotor;
  private static Elevator _instance = new Elevator();

  private static final double FEED_FORWARD_GAIN = 0.17427598;
  private static final double PROPORTIONAL_GAIN = 1.0;
  private static final double INTEGRAL_GAIN = 0;
  private static final int INTEGRAL_ZONE = 0;
  private static final double DERIVATIVE_GAIN = 0;
  private static final int CRUISE_VELOCITY = 1000;
  private static final int CRUISE_ACCELERATION = 4500;
  private static final int CAN_TIMEOUT_MILLISECONDS = 30;
  private double NATIVE_UNITS_TO_INCHES_CONVERSION = 242.7928;

  private double HOME_POSITION_NU = InchesToNativeUnits(0);
  private double CARGO_LEVEL_1_POSITION_NU = InchesToNativeUnits(0);
  private double CARGO_LEVEL_2_POSITION_NU = InchesToNativeUnits(0);
  private double CARGO_LEVEL_3_POSITION_NU = InchesToNativeUnits(0);
  private double HATCH_LEVEL_1_POSITION_NU = InchesToNativeUnits(0);
  private double HATCH_LEVEL_2_POSITION_NU = InchesToNativeUnits(0);
  private double HATCH_LEVEL_3_POSITION_NU = InchesToNativeUnits(0);


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
    _elevatorMasterMotor.configForwardSoftLimitEnable(false, 0);
    _elevatorMasterMotor.configReverseSoftLimitEnable(false, 0);

    // Configure brake mode
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

    // Configur velocity measurement
    _elevatorMasterMotor.configVelocityMeasurementPeriod(VelocityMeasPeriod.Period_5Ms, 0);
    _elevatorMasterMotor.configVelocityMeasurementWindow(32, 0);

    // Set up MotionMagic mode
    // SetPidSlotToUse("constr", MOVING_DOWN_PID_SLOT_INDEX)
    _elevatorMasterMotor.selectProfileSlot(0, 0);

    // Set closed loop gains
    _elevatorMasterMotor.config_kF(0, FEED_FORWARD_GAIN, CAN_TIMEOUT_MILLISECONDS);
    _elevatorMasterMotor.config_kP(0, PROPORTIONAL_GAIN, CAN_TIMEOUT_MILLISECONDS);
    _elevatorMasterMotor.config_kI(0, INTEGRAL_GAIN, CAN_TIMEOUT_MILLISECONDS);
    _elevatorMasterMotor.config_kD(0, DERIVATIVE_GAIN, CAN_TIMEOUT_MILLISECONDS);

    // Set accel and cruise velocities
    _elevatorMasterMotor.configMotionCruiseVelocity(CRUISE_VELOCITY, 0);
    _elevatorMasterMotor.configMotionAcceleration(CRUISE_ACCELERATION, 0);

  }

  public void MoveToPresetPosition(ELEVATOR_TARGET_POSITION presetPosition){
    if(get_hasElevatorBeenZeroed()){
      switch(presetPosition){
        case HOME:
          _targetElevatorPositionNU = HOME_POSITION_NU;
          break;
        case CARGO_LEVEL_1:
          _targetElevatorPositionNU = CARGO_LEVEL_1_POSITION_NU;
          break;
        case CARGO_LEVEL_2:
          _targetElevatorPositionNU = CARGO_LEVEL_2_POSITION_NU;
          break;
        case CARGO_LEVEL_3:
          _targetElevatorPositionNU = CARGO_LEVEL_3_POSITION_NU;
          break;
        case HATCH_LEVEL_1:
          _targetElevatorPositionNU = HATCH_LEVEL_1_POSITION_NU;
          break;
        case HATCH_LEVEL_2:
          _targetElevatorPositionNU = HATCH_LEVEL_2_POSITION_NU;
          break;
        case HATCH_LEVEL_3:
          _targetElevatorPositionNU = HATCH_LEVEL_3_POSITION_NU;
          break;
      }
    }
    _elevatorMasterMotor.set(ControlMode.MotionMagic, _targetElevatorPositionNU);
  }

  private boolean get_hasElevatorBeenZeroed() {
    return _hasElevatorBeenZeroed;
  }

  public void zeroElevatorMotorEncoder() {
    if (isBottomElevatorLimitSwitchClosed()) {
      _elevatorMasterMotor.setSelectedSensorPosition(0);
      _hasElevatorBeenZeroed = true;
    }
  }

  public int get_ElevatorPos() {
    return _elevatorMasterMotor.getSelectedSensorPosition(0);
  }

  public boolean isBottomElevatorLimitSwitchClosed() {
    return _elevatorMasterMotor.getSensorCollection().isRevLimitSwitchClosed();
  }

  public double NativeUnitsToInches(double nativeUnitsMeasure) {
    double inches = nativeUnitsMeasure / NATIVE_UNITS_TO_INCHES_CONVERSION;
    return inches;
  }

  public double InchesToNativeUnits(double inchesMeasure) {
    double nativeUnits = inchesMeasure * NATIVE_UNITS_TO_INCHES_CONVERSION;
    return Math.round(nativeUnits);
  }

  @Override
  public void initDefaultCommand() {
    // Set the default command for a subsystem here.
    // setDefaultCommand(new MySpecialCommand());
  }

  @Override
  public void updateLogData(LogDataBE logData) {

  }

  @Override
  public void updateDashboard() {
    SmartDashboard.putNumber("elevator pos", get_ElevatorPos());
    SmartDashboard.putNumber("elevator:inches", NativeUnitsToInches(get_ElevatorPos()));
    SmartDashboard.putNumber("elevator:native units", get_ElevatorPos());
  }

  

}
