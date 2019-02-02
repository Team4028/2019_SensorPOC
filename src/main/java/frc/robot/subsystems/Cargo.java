/*----------------------------------------------------------------------------*/
/* Copyright (c) 2018 FIRST. All Rights Reserved.                             */
/* Open Source Software - may be modified and shared by FRC teams. The code   */
/* must be accompanied by the FIRST BSD license file in the root directory of */
/* the project.                                                               */
/*----------------------------------------------------------------------------*/

package frc.robot.subsystems;

import com.ctre.phoenix.motorcontrol.ControlMode;
import com.ctre.phoenix.motorcontrol.LimitSwitchNormal;
import com.ctre.phoenix.motorcontrol.LimitSwitchSource;
import com.ctre.phoenix.motorcontrol.can.VictorSPX;

import edu.wpi.first.wpilibj.DoubleSolenoid;
import edu.wpi.first.wpilibj.Servo;
import edu.wpi.first.wpilibj.DoubleSolenoid.Value;
import edu.wpi.first.wpilibj.command.Subsystem;
import frc.robot.RobotMap;
import frc.robot.interfaces.IBeakSquadSubsystem;
import frc.robot.util.LogDataBE;

/**
 * Cargo (Ball) Subsystem Motors on CAN VictorSRX Motor Controllers Motor will
 * primarily run in vbus mode to infeed & outfeed the ball
 * 
 * Student Lead:
 */
public class Cargo extends Subsystem implements IBeakSquadSubsystem {

  VictorSPX _infeedMtr;
  private DoubleSolenoid _beakSolenoid;
  private DoubleSolenoid _punchSolenoid;
  private DoubleSolenoid _inOutSolenoid;
  private Servo _infeedServo;
  private boolean _isBeakOpen;
  private static final Value INIT_MOVE_MECHANISM = DoubleSolenoid.Value.kForward;
  private static final Value INIT_NOT_MOVED = DoubleSolenoid.Value.kReverse;
  private static final Value OUTWARD_BEAK_POSITION = DoubleSolenoid.Value.kForward;
  private static final Value INWARD_BEAK_POSITION = DoubleSolenoid.Value.kReverse;
  private static final Value SOLENOID_PUSHED = DoubleSolenoid.Value.kForward;
  private static final Value SOLENOID_READY_TO_BE_PUSHED = DoubleSolenoid.Value.kReverse;


  public enum BEAK_POSITION
  {
    UNDEFINED, 
    OPEN,
    CLOSED
  }
  //=====================================================================================
	// Define Singleton Pattern
	//=====================================================================================
	private static Cargo _instance = new Cargo();
	
	public static Cargo getInstance() {
    return _instance;
	}
	
	// private constructor for singleton pattern
  private Cargo() 
  {
    _infeedMtr = new VictorSPX(RobotMap.CARGO_VICTOR_ADDR);
    _infeedMtr.configFactoryDefault();
    _infeedMtr.configForwardLimitSwitchSource(LimitSwitchSource.Deactivated, LimitSwitchNormal.Disabled);
   // _infeedMtr.configReverseLimitSwitchSource(LimitSwitchSource.Deactivated, LimitSwitchNormal.Disabled);
    _beakSolenoid = new DoubleSolenoid(RobotMap.PCM_FORWARD_BEAK_SOLENOID_PORT,RobotMap.PCM_REVERSE_BEAK_SOLENOID_PORT);
    _punchSolenoid = new DoubleSolenoid(RobotMap.PCM_FORWARD_PUNCH_SOLENOID_PORT, RobotMap.PCM_REVERSE_PUNCH_SOLENOID_PORT);
    _inOutSolenoid = new DoubleSolenoid(RobotMap.PCM_FORWARD_INOUT_SOLENOID_PORT, RobotMap.PCM_REVERSE_INOUT_SOLENOID_PORT);

    setDefultPosition();
    
  }

  // Put methods for controlling this subsystem
  // here. Call these from Commands.
  
  public void setMotorSpeed (double driveSpeed)
    {
        double Speed = (.7 * driveSpeed);
        _infeedMtr.set(ControlMode.PercentOutput, Speed);
    }    
  public void setDefultPosition()
  {
    _beakSolenoid.set(INWARD_BEAK_POSITION);
    _inOutSolenoid.set(INIT_NOT_MOVED);
    _punchSolenoid.set(SOLENOID_PUSHED);
  }

  
  public void moveEntireMechanismForward()
    {
      if(_isBeakOpen= false)
      {
        Value currentMechPos = _inOutSolenoid.get();
        if(currentMechPos == INIT_NOT_MOVED)
        {
          _inOutSolenoid.set(INIT_MOVE_MECHANISM);
        }
        else
        {
          _inOutSolenoid.set(INIT_NOT_MOVED);
        }
      }
    }
  public void toggleBeakPlacement()
    {
      
      Value currentBeakPos = _beakSolenoid.get();
      if(currentBeakPos == OUTWARD_BEAK_POSITION)
      {
        _beakSolenoid.set(INWARD_BEAK_POSITION);
        _isBeakOpen = false;
      }
      else
      {
        _beakSolenoid.set(OUTWARD_BEAK_POSITION);
        _isBeakOpen = true;
      }
    }
    
    public void pushHatchOffBeak()
    {
      if(_isBeakOpen = false)
      {
        Value currentPushPos = _punchSolenoid.get();
        if(currentPushPos == SOLENOID_PUSHED)
        {
          _punchSolenoid.set(SOLENOID_READY_TO_BE_PUSHED);
          
        }
        else
        {
          _punchSolenoid.set(SOLENOID_PUSHED);
        }
      }
    }
    public void setBeak(BEAK_POSITION beakPosition)
    {
      if(beakPosition == BEAK_POSITION.OPEN)
      {
        _beakSolenoid.set(OUTWARD_BEAK_POSITION);
        _isBeakOpen = true;
      }
      else if(beakPosition == BEAK_POSITION.CLOSED)
      {
        _isBeakOpen = false;
        _beakSolenoid.set(INWARD_BEAK_POSITION);
      }
    
    }
    public void servoStartMatch()
    {
      _infeedServo.set(1);
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
