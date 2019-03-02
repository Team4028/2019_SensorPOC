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
  private DoubleSolenoid _beakOpenCloseSolenoid;
  private DoubleSolenoid _punchSolenoid;
  private DoubleSolenoid _beakInOutSolenoid;
  private DoubleSolenoid _bucketSolenoid;
  private Servo _infeedServo;
  private DigitalInput _hatchLimitSwitch;
  private static final Value MECHANISM_EXTENDED = DoubleSolenoid.Value.kForward;
  private static final Value MECHANISM_RETRACTED = DoubleSolenoid.Value.kReverse;
  private static final Value BEAK_OPEN = DoubleSolenoid.Value.kForward;
  private static final Value BEAK_CLOSE = DoubleSolenoid.Value.kReverse;
  private static final Value PUNCH_IN = DoubleSolenoid.Value.kForward;
  private static final Value PUNCH_OUT = DoubleSolenoid.Value.kReverse;
  private static final Value BUCKET_EXTENDED = DoubleSolenoid.Value.kForward;
  private static final Value BUCKET_RETRACTED= DoubleSolenoid.Value.kReverse;


  // =================================================================================================================
  // Define Enums for the Cargo
  public enum BEAK_OPENCLOSE_POSITION {
    UNDEFINED, 
    OPEN,
    CLOSED
  }

  public enum PUNCH_POSITION {
    UNDEFINED,
    OUT,
    IN

  }

  public enum BEAK_INOUT_POSITION {
    UNDEFINED,
    EXTENDED,
    RETRACTED
  }
  public enum BUCKET_POSITION {
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
    _beakOpenCloseSolenoid = new DoubleSolenoid(RobotMap.PCM_FORWARD_BEAK_SOLENOID_PORT,RobotMap.PCM_REVERSE_BEAK_SOLENOID_PORT);
    _punchSolenoid = new DoubleSolenoid(RobotMap.PCM_FORWARD_PUNCH_SOLENOID_PORT, RobotMap.PCM_REVERSE_PUNCH_SOLENOID_PORT);
    _beakInOutSolenoid = new DoubleSolenoid(RobotMap.PCM_REVERSE_INOUT_SOLENOID_PORT, RobotMap.PCM_FORWARD_INOUT_SOLENOID_PORT);
    _bucketSolenoid = new DoubleSolenoid(RobotMap.PCM_FORWARD_BUCKET_SOLENOID_PORT, RobotMap.PCM_REVERSE_BUCKET_SOLENOID_PORT);

    _hatchLimitSwitch = new DigitalInput(RobotMap.CARGO_LIMIT_SWITCH_DIO_PORT);
    
    //Hatch Limit Switch
    _hatchLimitSwitch = new DigitalInput(RobotMap.CARGO_LIMIT_SWITCH_DIO_PORT);

    setCargoDefultPosition();
  }
  
  public void setMotorSpeed (double driveSpeed) {
    if (get_isHatchAquired() == true)
    {
      double Speed = (.7 * driveSpeed);
    _infeedMtr.set(ControlMode.PercentOutput, Speed);;
    }
  } 

  public void setCargoDefultPosition() {
    _beakOpenCloseSolenoid.set(BEAK_CLOSE);
    _beakInOutSolenoid.set(MECHANISM_RETRACTED);
    _punchSolenoid.set(PUNCH_IN);
    _bucketSolenoid.set(BUCKET_RETRACTED);
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
  public void setBeakOpenClose(BEAK_OPENCLOSE_POSITION desiredBeakPosition) {
    Value currentBeakInOutPos = _beakInOutSolenoid.get();
    Value currentPunchPos = _punchSolenoid.get();
    
    if (desiredBeakPosition == BEAK_OPENCLOSE_POSITION.CLOSED) {
          _beakOpenCloseSolenoid.set(BEAK_CLOSE);
    }
    else if (desiredBeakPosition == BEAK_OPENCLOSE_POSITION.OPEN) {
      if(currentPunchPos == PUNCH_IN && currentBeakInOutPos == BEAK_OPEN) {
          _beakOpenCloseSolenoid.set(BEAK_OPEN);
      } else {
        DriverStation.reportWarning("BEAK SAFETY INTERLOCK U SUC", false);
      }
    }
  }

  public void setPunch(PUNCH_POSITION punchPosition) { 
    Value currentBeakInOutPos = _beakInOutSolenoid.get();
    Value currentBeakOpenClosePos = _beakOpenCloseSolenoid.get();

    if (punchPosition == PUNCH_POSITION.IN) {
      _punchSolenoid.set(PUNCH_IN);
      
    } 
    else if (punchPosition == PUNCH_POSITION.OUT) {
      if(currentBeakOpenClosePos == BEAK_CLOSE && currentBeakInOutPos == BEAK_OPEN) {
        _punchSolenoid.set(PUNCH_OUT);
      } else {
        DriverStation.reportWarning("PUNCH SAFETY INTERLOCK U SUC", false);
      }
    }
  }

  public void setBeakInOut(BEAK_INOUT_POSITION beakInOutPosition) {
    Value currentBeakPos = _beakOpenCloseSolenoid.get();
    if (beakInOutPosition == BEAK_INOUT_POSITION.EXTENDED) {
      _beakInOutSolenoid.set(MECHANISM_EXTENDED);
    
    } 
    else if (beakInOutPosition == BEAK_INOUT_POSITION.RETRACTED) {
      if(currentBeakPos== BEAK_CLOSE) 
      {
           _beakInOutSolenoid.set(MECHANISM_RETRACTED);
        } else {
          DriverStation.reportWarning("MECHANISM SAFETY INTERLOCK U SUC", false);
        }
      }
    }
    public void setBucket(BUCKET_POSITION bucketPosition) {
  
      if (bucketPosition == BUCKET_POSITION.EXTENDED) {
        _bucketSolenoid.set(BUCKET_EXTENDED);
      
      } 
      else if (bucketPosition == BUCKET_POSITION.RETRACTED) 
      {
        _bucketSolenoid.set(BUCKET_RETRACTED);
      }
    
  }

  // =====================================
  public void toggleBeakInOut() {
    Value currentBeakInOutPos = _beakInOutSolenoid.get();
    if (currentBeakInOutPos == MECHANISM_RETRACTED) {
      setBeakInOut(BEAK_INOUT_POSITION.EXTENDED);
    } else {
      setBeakInOut(BEAK_INOUT_POSITION.RETRACTED);
    }
  }

  public void toggleBeakOpenClose() {
    Value currentBeakPos = _beakOpenCloseSolenoid.get();
    if (currentBeakPos == BEAK_CLOSE) {
      setBeakOpenClose(BEAK_OPENCLOSE_POSITION.OPEN);
    } else {
      setBeakOpenClose(BEAK_OPENCLOSE_POSITION.CLOSED);
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

  public void toggleBucket()
  {
    Value currentBucketPos = _bucketSolenoid.get();
    if (currentBucketPos == BUCKET_EXTENDED)
    {
      setBucket(BUCKET_POSITION.RETRACTED);
    }
    else
    {
      setBucket(BUCKET_POSITION.EXTENDED);
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
    Value currentBeakPos = _beakOpenCloseSolenoid.get();
    if (currentBeakPos == BEAK_CLOSE) {
    return "Beak Closed";
    } else {
      return "Beak Open";
    }
  }

  public boolean get_isBeakOpen() {
    
    if(_beakOpenCloseSolenoid.get() == BEAK_OPEN){
      return true;
    } else {
      return false;
    }
  }
  
  public boolean get_isPunchOut() {
    
    if(_punchSolenoid.get() == PUNCH_OUT){
      return true;
    } else {
      return false;
    }
  }

  public boolean get_isBeakOut() {
    
    if(_beakInOutSolenoid.get() == BEAK_OPEN){
      return true;
    } else {
      return false;
    }
  }

  public boolean get_isBucketExtended() {
    
    if(_bucketSolenoid.get() == BUCKET_EXTENDED){

      return true;
    } else {
      return false;
    }
  }
  
  public boolean get_isHatchAquired()
  {
    return !_hatchLimitSwitch.get();
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
    SmartDashboard.putBoolean("Cargo:IsBucketOut", get_isBucketExtended());
    SmartDashboard.putBoolean("Cargo:IsBeakOut", get_isBeakOut());
    SmartDashboard.putBoolean("Cargo:IsBeakOpen", get_isBeakOpen());
    SmartDashboard.putBoolean("Cargo:IsPunchOut", get_isPunchOut());
    
  }
}