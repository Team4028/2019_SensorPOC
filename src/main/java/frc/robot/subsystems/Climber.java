package frc.robot.subsystems;

import javax.lang.model.util.ElementScanner6;

import com.ctre.phoenix.motorcontrol.ControlMode;
import com.ctre.phoenix.motorcontrol.FeedbackDevice;
import com.ctre.phoenix.motorcontrol.LimitSwitchNormal;
import com.ctre.phoenix.motorcontrol.LimitSwitchSource;
import com.ctre.phoenix.motorcontrol.NeutralMode;
import com.ctre.phoenix.motorcontrol.StatusFrame;
import com.ctre.phoenix.motorcontrol.can.TalonSRX;
import com.ctre.phoenix.motorcontrol.can.VictorSPX;

import edu.wpi.first.wpilibj.command.Subsystem;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import frc.robot.interfaces.IBeakSquadSubsystem;
import frc.robot.util.LogDataBE;
import frc.robot.RobotMap;

/**
 * Climber Subsystem
 *  1 Motor on CAN TalonSPX Motor Controller  w/ encoder + l/s?
 *  1 Motor on CAN VictorSRX Motor Controller open loop
 * 
 * Student Lead: Peter Nicholas
 */
public class Climber extends Subsystem implements IBeakSquadSubsystem {
	// =================================================================================================================
  // define class level working variables
  TalonSRX _liftMtr;
  VictorSPX _driveMtr;
  private static final double INCHES_TO_NU_CONVERSION_NUMBER = .0013729128051576489005976;
  private static final double THIRD_LEVEL_CLIMBER_MEASUREMENT_IN_NU = -20299;

  //=====================================================================================
	// Define Singleton Pattern
	//=====================================================================================
	private static Climber _instance = new Climber();
	
	public static Climber getInstance() {
		return _instance;
	}
	
  // private constructor for singleton pattern
  
	private Climber() {
    _liftMtr = new TalonSRX(RobotMap.CLIMBER_LIFT_CAN_ADDR);

    //Configure Limit Switches
    _liftMtr.configForwardLimitSwitchSource(LimitSwitchSource.FeedbackConnector, LimitSwitchNormal.NormallyClosed);
    _liftMtr.configReverseLimitSwitchSource(LimitSwitchSource.Deactivated, LimitSwitchNormal.Disabled);

    // Turn of all soft limits
   _liftMtr.configForwardSoftLimitEnable(false, 0);
   _liftMtr.configReverseSoftLimitEnable(false, 0);

    //Configure brake mode
   _liftMtr.setNeutralMode(NeutralMode.Brake);

    // Configure Encoder
   _liftMtr.configSelectedFeedbackSensor(FeedbackDevice.CTRE_MagEncoder_Relative, 0, 0);
   _liftMtr.setSensorPhase(true);
   _liftMtr.setStatusFramePeriod(StatusFrame.Status_2_Feedback0, 5, 0);


    _driveMtr = new VictorSPX(RobotMap.CLIMBER_DRIVE_CAN_ADDR);
   // _driveMtr.configFactoryDefault();
  }

  public void liftClimber(double liftSpeed){
    _liftMtr.set(ControlMode.PercentOutput, liftSpeed);
  }

  public void driveClimber(double driveSpeed){
    _driveMtr.set(ControlMode.PercentOutput, driveSpeed);
  }
  

  public void setClimberPosition(){
   // if (_liftMtr.getSelectedSensorPosition() < THIRD_LEVEL_CLIMBER_MEASUREMENT_IN_NU) {
      _liftMtr.set(ControlMode.PercentOutput, .5);
/*} else {
      System.out.println("Are you moving yet?");
      _liftMtr.set(ControlMode.PercentOutput, 0);
    }*/
  }
  public boolean isClimberatTop()
  {
    return !_liftMtr.getSensorCollection().isFwdLimitSwitchClosed();
  }
  public void zeroClimber()
  {
    _liftMtr.setSelectedSensorPosition(0);
  }
  public void HoldClimber()
  {
    _liftMtr.config_kF(0, 0.5);
    _liftMtr.config_kP(0,0.5);
    _liftMtr.config_kI(0,0);
    _liftMtr.config_kD(0,0);
    _liftMtr.set(ControlMode.MotionMagic, getNativeUnits());
  }

  public double inchesToNativeUnits(double inches){
    return inches/ INCHES_TO_NU_CONVERSION_NUMBER;
  }
  public double nativeUnitsToInches(double nativeUnits){
    return nativeUnits * .0013729128051576489005976;
  }


	public double getPositionInches(){
		return nativeUnitsToInches(getNativeUnits());
	}
  public double getNativeUnits(){
    return  _liftMtr.getSelectedSensorPosition();
  }


  @Override
  public void initDefaultCommand() {
    // Set the default command for a subsystem here.
    // setDefaultCommand(new MySpecialCommand());
  }

	// ===============================================================================================================
	// Default Command
	// ===============================================================================================================
  @Override
  public void updateLogData(LogDataBE logData){
  }

  @Override
  public void updateDashboard() {
    SmartDashboard.putNumber("Encoder reading in NU", _liftMtr.getSelectedSensorPosition());
    SmartDashboard.putNumber("Lift motor command", _liftMtr.getMotorOutputPercent());
    SmartDashboard.putNumber("apparent encoder value", _liftMtr.getSelectedSensorPosition());
  }
}