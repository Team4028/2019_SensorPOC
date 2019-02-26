package frc.robot.subsystems;

import com.ctre.phoenix.motorcontrol.ControlMode;
import com.ctre.phoenix.motorcontrol.LimitSwitchNormal;
import com.ctre.phoenix.motorcontrol.LimitSwitchSource;
import com.ctre.phoenix.motorcontrol.can.VictorSPX;

import edu.wpi.first.wpilibj.DigitalInput;
import edu.wpi.first.wpilibj.DoubleSolenoid;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.Servo;
import edu.wpi.first.wpilibj.DoubleSolenoid.Value;
import edu.wpi.first.wpilibj.command.Subsystem;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import frc.robot.RobotMap;
import frc.robot.interfaces.IBeakSquadSubsystem;
import frc.robot.util.LogDataBE;

/**
 * Cargo (Ball) Subsystem Motors on CAN VictorSRX Motor Controllers Motor will
 * primarily run in vbus mode to infeed & outfeed the ball
 * 
 * Student Lead: Parker Johnson
 */
public class Cargo extends Subsystem implements IBeakSquadSubsystem {

  // =================================================================================================================
  // define class level working variables
  VictorSPX _infeedMtr;
  private DoubleSolenoid _beakSolenoid;
  private DoubleSolenoid _punchSolenoid;
  private DoubleSolenoid _mechansimSolenoid;
  private DoubleSolenoid _releaseSolenoid;
  private Servo _infeedServo;
  private DigitalInput _hatchLimitSwitch;
  private static final Value MECHANISM_EXTENDED = DoubleSolenoid.Value.kForward;
  private static final Value MECHANISM_RETRACTED = DoubleSolenoid.Value.kReverse;
  private static final Value BEAK_OPEN = DoubleSolenoid.Value.kForward;
  private static final Value BEAK_CLOSE = DoubleSolenoid.Value.kReverse;
  private static final Value PUNCH_IN = DoubleSolenoid.Value.kForward;
  private static final Value PUNCH_OUT = DoubleSolenoid.Value.kReverse;
  private static final Value RELEASE_EXTENDED = DoubleSolenoid.Value.kForward;
  private static final Value RELEASE_RETRACTED= DoubleSolenoid.Value.kReverse;


  // =================================================================================================================
  // Define Enums for the Cargo
  public enum BEAK_POSITION {
    UNDEFINED, 
    OPEN,
    CLOSED
  }

  public enum PUNCH_POSITION {
    UNDEFINED,
    OUT,
    IN
  }

  public enum MECHANISM_POSITION {
    UNDEFINED,
    EXTENDED,
    RETRACTED
  }
  public enum RELEASE_POSITION {
    UNDEFINED,
    EXTENDED,
    RETRACTED
  }
  
  //=====================================================================================
	// Define Singleton Pattern
	//=====================================================================================
	private static Cargo _instance = new Cargo();
	
	public static Cargo getInstance() {
    return _instance;
	}
	
	// private constructor for singleton pattern
  private Cargo() {
    //Infeed Motor
    _infeedMtr = new VictorSPX(RobotMap.INFEED_DRIVE_CAN_ADDR);
    _infeedMtr.configFactoryDefault();
    _infeedMtr.configForwardLimitSwitchSource(LimitSwitchSource.Deactivated, LimitSwitchNormal.Disabled);

    //Hatch Solenoids
    _beakSolenoid = new DoubleSolenoid(RobotMap.PCM_FORWARD_BEAK_SOLENOID_PORT,RobotMap.PCM_REVERSE_BEAK_SOLENOID_PORT);
    _punchSolenoid = new DoubleSolenoid(RobotMap.PCM_FORWARD_PUNCH_SOLENOID_PORT, RobotMap.PCM_REVERSE_PUNCH_SOLENOID_PORT);
    _mechansimSolenoid = new DoubleSolenoid(RobotMap.PCM_REVERSE_INOUT_SOLENOID_PORT, RobotMap.PCM_FORWARD_INOUT_SOLENOID_PORT);
    _releaseSolenoid = new DoubleSolenoid(RobotMap.PCM_FORWARD_RELEASE_SOLENOID_PORT, RobotMap.PCM_REVERSE_RELEASE_SOLENOID_PORT);
    
    //Hatch Limit Switch
    _hatchLimitSwitch = new DigitalInput(RobotMap.CARGO_LIMIT_SWITCH_DIO_PORT);

    setCargoDefultPosition();
  }
  
  public void setMotorSpeed (double driveSpeed) {
    double Speed = (.7 * driveSpeed);
    _infeedMtr.set(ControlMode.PercentOutput, Speed);
  } 

  public void setCargoDefultPosition() {
    _beakSolenoid.set(BEAK_CLOSE);
    _mechansimSolenoid.set(MECHANISM_EXTENDED);
    _punchSolenoid.set(PUNCH_IN);
    _releaseSolenoid.set(RELEASE_RETRACTED);
  }

  // ===================================== 
  /*
      These methods set the values of the solenoids that are used. The toggle method calls them and they chanfge accordingly. 
      Safety interlock has been added so that punch (the function that pushes the hatch off of the beak) cannot be run
      when the beak has a hatch and the mechanism is retracted. The beak cannot be opened if the 
      sliding mechnaism isnt extended and if the punch is extended. The mechanism cannot slide when
      the beak is open :)
  */
  //=======================================
  public void setBeak(BEAK_POSITION desiredBeakPosition) {
    Value currentMechPos = _mechansimSolenoid.get();
    Value currentPunchPos = _punchSolenoid.get();
    
    if (desiredBeakPosition == BEAK_POSITION.CLOSED) {
          _beakSolenoid.set(BEAK_CLOSE);
    }
    else if (desiredBeakPosition == BEAK_POSITION.OPEN) {
      if(currentPunchPos == PUNCH_IN && currentMechPos == MECHANISM_EXTENDED) {
          _beakSolenoid.set(BEAK_OPEN);
      } else {
        DriverStation.reportWarning("BEAK SAFETY INTERLOCK U SUC", false);
      }
    }
  }

  public void setPunch(PUNCH_POSITION punchPosition) { 
    Value currentMechPos = _mechansimSolenoid.get();
    Value currentBeakPos = _beakSolenoid.get();

    if (punchPosition == PUNCH_POSITION.IN) {
      _punchSolenoid.set(PUNCH_IN);
      
    } 
    else if (punchPosition == PUNCH_POSITION.OUT) {
      if(currentBeakPos== BEAK_CLOSE && currentMechPos == MECHANISM_EXTENDED) {
           _punchSolenoid.set(PUNCH_OUT);
        } else {
          DriverStation.reportWarning("PUNCH SAFETY INTERLOCK U SUC", false);
        }
    }
  }

  public void setMechanism(MECHANISM_POSITION mechanismPosition) {
    Value currentBeakPos = _beakSolenoid.get();
    if (mechanismPosition == MECHANISM_POSITION.EXTENDED) {
      _mechansimSolenoid.set(MECHANISM_EXTENDED);
    
    } 
    else if (mechanismPosition == MECHANISM_POSITION.RETRACTED) {
      if(currentBeakPos== BEAK_CLOSE) {
           _mechansimSolenoid.set(MECHANISM_RETRACTED);
        } else {
          DriverStation.reportWarning("MECHANISM SAFETY INTERLOCK U SUC", false);
        }
      }
    }
    public void setRelease(RELEASE_POSITION releasePosition) {
  
      if (releasePosition == RELEASE_POSITION.EXTENDED) {
        _releaseSolenoid.set(RELEASE_EXTENDED);
      
      } 
      else if (releasePosition == RELEASE_POSITION.RETRACTED) 
      {
        _releaseSolenoid.set(MECHANISM_RETRACTED);
      }
    
  }

  // =====================================
  public void toggleMechanism() {
    Value currentMechPos = _mechansimSolenoid.get();
    if (currentMechPos == MECHANISM_RETRACTED) {
      setMechanism(MECHANISM_POSITION.EXTENDED);
    } else {
      setMechanism(MECHANISM_POSITION.RETRACTED);
    }
  }

  public void toggleBeakPlacement() {
    Value currentBeakPos = _beakSolenoid.get();
    if (currentBeakPos == BEAK_CLOSE) {
      setBeak(BEAK_POSITION.OPEN);
    } else {
      setBeak(BEAK_POSITION.CLOSED);
    }
  }

  public void togglePunch() {
    Value currentPunchPos = _punchSolenoid.get();
    if (currentPunchPos == PUNCH_OUT) {
      setPunch(PUNCH_POSITION.IN);
    } else {
      setPunch(PUNCH_POSITION.OUT);
    }
  }

  public void toggleRelease()
  {
    Value currentReleasePos = _releaseSolenoid.get();
    if (currentReleasePos == RELEASE_EXTENDED)
    {
      setRelease(RELEASE_POSITION.RETRACTED);
    }
    else
    {
      setRelease(RELEASE_POSITION.EXTENDED);
    }
    DriverStation.reportWarning("Release is Running", false);
  }
  public void servoStartMatch(){
    _infeedServo.set(1);
  }

  // ===============================================================================================================
	// Expose Properties of Elevator
	// ===============================================================================================================
  private String get_BeakPosition(){
    Value currentBeakPos = _beakSolenoid.get();
    if (currentBeakPos == BEAK_CLOSE) {
    return "Beak Closed";
    } else {
      return "Beak Open";
    }
  }

  private String get_PunchPosition(){
    Value currentPunchPos = _punchSolenoid.get();
    if (currentPunchPos == PUNCH_OUT) {
      return "Punch Out";
    } else {
      return "Punch In";
    }
  }

  private String get_ReleasePosition(){
    Value currentReleasePos = _releaseSolenoid.get();
    if (currentReleasePos == RELEASE_RETRACTED) {
      return "Release Retracted";
    } else {
      return "Release Extended";
    }
  }

  private String get_MechPosition(){
    Value currentMechPos = _mechansimSolenoid.get();
    if (currentMechPos == MECHANISM_RETRACTED) {
      return "Mechanism Retracted";
    } else {
      return "Mechanism Extended";
    }
  }

  public boolean get_IsBucketOut() {
    
    if(_releaseSolenoid.get() == RELEASE_EXTENDED){

      return true;
    } else {
      return false;
    }
  }

  public boolean get_HasHatch() {
    return _hatchLimitSwitch.get();
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
    SmartDashboard.putString("MechanismPos", get_MechPosition());
    SmartDashboard.putString("PunchPos", get_PunchPosition());
    SmartDashboard.putString("BeakPos", get_BeakPosition());
    SmartDashboard.putString("ReleasePos", get_ReleasePosition());
  }
}
