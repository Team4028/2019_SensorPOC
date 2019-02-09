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
import frc.robot.RobotMap;
 

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
  private static Elevator _instance = new Elevator();

  public static Elevator getInstance(){
    return _instance;
  }

  private Elevator(){
    _elevatorMasterMotor = new TalonSRX(RobotMap.ELEVATOR_MASTER_CAN_ADDR);
    _elevatorSlaveMotor = new TalonSRX(RobotMap.ELEVATOR_MASTER_CAN_ADDR);

    _elevatorSlaveMotor.follow(_elevatorMasterMotor);

    // Set motor phasing
    _elevatorMasterMotor.setInverted(false);

    // Configure Limit Switch
    _elevatorMasterMotor.configForwardLimitSwitchSource(LimitSwitchSource.FeedbackConnector, LimitSwitchNormal.NormallyClosed);
    _elevatorMasterMotor.configReverseLimitSwitchSource(LimitSwitchSource.FeedbackConnector, LimitSwitchNormal.NormallyClosed);

    // Turn of all soft limits
    _elevatorMasterMotor.configForwardSoftLimitEnable(false, 0);
    _elevatorMasterMotor.configReverseSoftLimitEnable(false, 0);

    //Configure brake mode
    _elevatorMasterMotor.setNeutralMode(NeutralMode.Brake);
    _elevatorSlaveMotor.setNeutralMode(NeutralMode.Brake);

    // Configure Elevator
    _elevatorMasterMotor.configSelectedFeedbackSensor(FeedbackDevice.CTRE_MagEncoder_Relative, 0, 0);
    _elevatorMasterMotor.setSensorPhase(true);
    _elevatorMasterMotor.setStatusFramePeriod(StatusFrame.Status_2_Feedback0, 5, 0);
    
    // peak/nominal output voltages for both directions for talons configuration
    _elevatorMasterMotor.configNominalOutputForward(0, 0);
    _elevatorMasterMotor.configNominalOutputReverse(0, 0);
    _elevatorMasterMotor.configPeakOutputForward(1, 0);
    _elevatorMasterMotor.configPeakOutputReverse(-1, 0);
    
    //Configur velocity measurement (2X of scan time, looper is 10 mS)
    _elevatorMasterMotor.configVelocityMeasurementPeriod(VelocityMeasPeriod.Period_5Ms, 0);
    _elevatorMasterMotor.configVelocityMeasurementWindow(32, 0);

    //Set up MotionMagic mode
    //SetPidSlotToUse("constr", MOVING_DOWN_PID_SLOT_INDEX)

    // Set closed loop gains
    //_elevatorMasterMotor.config_kF(MOVING_DOWN_PID_SLOT_INDEX, FEED_FORWARD_GAIN+DOWN, 0);
    //_elevatorMasterMotor.config_kP(MOVING_DOWN_PID_SLOT_INDEX, FEED_FORWARD_GAIN+DOWN, 0);
    //_elevatorMasterMotor.config_kI(MOVING_DOWN_PID_SLOT_INDEX, FEED_FORWARD_GAIN+DOWN, 0);
    //_elevatorMasterMotor.config_kD(MOVING_DOWN_PID_SLOT_INDEX, FEED_FORWARD_GAIN+DOWN, 0);

    //Set accel and cruise velocities
    //_elevatorMasterMotor.configMotionCruiseVelocity(UP_CRUISE_VELOCITY, 0);
    //_elevatorMasterMotor.configMotionCruiseAcceleration(TELEOP_UP_CRUISE_ACCELERATION, 0);
    
  }

  public void moveElevator(ELEVATOR_UP_OR_DOWN elevatorUpOrDown){
    switch(elevatorUpOrDown){
      case UP:
          _elevatorMasterMotor.set(ControlMode.PercentOutput, .05);
        break;
      case DOWN:
        _elevatorMasterMotor.set(ControlMode.PercentOutput, -.05);
        break;
    }
  }


  @Override
  public void initDefaultCommand() {
    // Set the default command for a subsystem here.
    // setDefaultCommand(new MySpecialCommand());
  }
}
