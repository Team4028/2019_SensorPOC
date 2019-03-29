package frc.robot;

import java.math.BigDecimal;
import java.text.DecimalFormat;
import java.util.Date;

import com.ctre.phoenix.motorcontrol.ControlMode;
import com.ctre.phoenix.motorcontrol.NeutralMode;

import edu.wpi.first.wpilibj.TimedRobot;
import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj.command.Scheduler;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import frc.robot.auton.pathfollowing.Paths;
import frc.robot.commands.auton.StartAcquireHatch;
import frc.robot.commands.chassis.DriveWithControllers;
import frc.robot.commands.chassis.StopChassis;
import frc.robot.commands.climber.ZeroClimber;
import frc.robot.commands.elevator.ZeroElevatorEncoder;
import frc.robot.commands.infeed.AcquireHatch;
import frc.robot.commands.infeed.SendBucketIn;
import frc.robot.commands.infeed.SendBucketOut;
import frc.robot.commands.vision.ChoosePipeline;
import frc.robot.interfaces.IVisionSensor;
import frc.robot.sensors.GyroNavX;

import frc.robot.sensors.VisionLL;
import frc.robot.sensors.VisionLL.LIMELIGHT_PIPELINE;
import edu.wpi.first.wpilibj.command.Command;
import edu.wpi.first.wpilibj.command.CommandGroup;
import frc.robot.sensors.AirCompressor;
import frc.robot.sensors.DistanceRev2mSensor;
import frc.robot.sensors.StoredPressureSensor;
import frc.robot.sensors.SwitchableCameraServer;
import frc.robot.subsystems.Cargo;
import frc.robot.subsystems.Chassis;
import frc.robot.subsystems.Climber;
import frc.robot.subsystems.Elevator;
import frc.robot.subsystems.Chassis.ChassisState;
import frc.robot.util.DataLogger;
import frc.robot.util.GeneralUtilities;
import frc.robot.util.LogDataBE;
import frc.robot.util.MovingAverage;
import frc.robot.ux.AutonChoosers;
import frc.robot.ux.LEDController;
import frc.robot.ux.OI;

/**
 * The VM is configured to automatically run this class, and to call the
 * functions corresponding to each mode, as described in the TimedRobot
 * documentation. If you change the name of this class or the package after
 * creating this project, you must also update the build.gradle file in the
 * project.
 */
public class Robot extends TimedRobot {

	private static final String ROBOT_NAME = "2019 DeepSpace SensorPOC";

  // create instance of each Subsystem (singleton)
  //  Note: add each one to the outputAllToDashboard & logAllData methods below

  // sensors
  private DistanceRev2mSensor _distanceRev2mSensor = DistanceRev2mSensor.getInstance();
  private StoredPressureSensor _pressureSensor = null; //StoredPressureSensor.getInstance();
  private SwitchableCameraServer _cameraServer = SwitchableCameraServer.getInstance();
  private AirCompressor _compressor = AirCompressor.get_instance();

  private IVisionSensor _vision = VisionLL.getInstance();      // Limelight
  //private IVisionSensor _vision = VisionIP.getInstance();   // IPhone
  private GyroNavX _navX = GyroNavX.getInstance();

  // ux
  private LEDController _leds = LEDController.getInstance();
  private AutonChoosers _autonChoosers = AutonChoosers.getInstance();

  // subsystems
  private Chassis _chassis = Chassis.getInstance();
  private Cargo _cargo = Cargo.getInstance();
  private Climber _climber = Climber.getInstance();
  private Elevator _elevator = Elevator.getInstance();

  // class level working variables
	private DataLogger _dataLogger = null;
	private String _buildMsg = "?";
 	long _lastScanEndTimeInMSec;
 	long _lastDashboardWriteTimeMSec;
	MovingAverage _scanTimeSamples;
  public double _startTime;
  OI _oi = OI.getInstance();
  boolean hasAutonBeenScheduled;
  boolean hasClimberZeroed;


  /********************************************************************************************
   * This function is run when the robot is first started up and should be used
   * for any initialization code.
   ********************************************************************************************/
  @Override
  public void robotInit() {
    _buildMsg = GeneralUtilities.WriteBuildInfoToDashboard(ROBOT_NAME);
    Paths.buildPaths();
    _leds.mvrCompPrettyColors();
  }

  /********************************************************************************************
   * Autonomous Mode
   ********************************************************************************************/
  /**
   * This function is called 1x when the robot is 1st enabled autonomous mode
   */
  @Override
  public void autonomousInit() {
    // CommandGroup startAcquireHatch = new StartAcquireHatch();
    // startAcquireHatch.start();
    Paths.havePathsBuilt=false;
    Paths.buildPaths();
    _chassis.initiateRobotState();
    _chassis.zeroSensors();
    _chassis.stop();
    _chassis.setBrakeMode(NeutralMode.Brake);
    _scanTimeSamples = new MovingAverage(20);
    _lastDashboardWriteTimeMSec = new Date().getTime(); // snapshot time to control spamming
    _dataLogger = GeneralUtilities.setupLogging("Auton"); // init data logging	

    Chassis._autoStartTime = Timer.getFPGATimestamp();
    if(!_elevator.get_hasElevatorBeenZeroed()){
      Command zeroElevatorCommand = new ZeroElevatorEncoder();
      zeroElevatorCommand.start();
    }
    hasAutonBeenScheduled=false;
    hasClimberZeroed=false;
  }

  /**
   * This function is called periodically during autonomous mode.
   */
  @Override
  public void autonomousPeriodic() 
  {
    if(!hasClimberZeroed)
    {
      Command zeroClimber = new ZeroClimber();
      zeroClimber.start();
      hasClimberZeroed=true;
    }

    if(Paths.havePathsBuilt)
    {
      if(!hasAutonBeenScheduled && Timer.getFPGATimestamp()-_chassis._autoStartTime>0.2)
      {
        CommandGroup auton = _autonChoosers.getSelectedAuton();
        System.out.println(auton);
        hasAutonBeenScheduled=true;
        auton.start();
      }
    }
    Scheduler.getInstance().run();
    _chassis.updateChassis(Timer.getFPGATimestamp());
    _vision.turnOnLEDs();
    if(_chassis.getForcedAutonFinish())
    {
      _autonChoosers.getSelectedAuton().cancel();
      Scheduler.getInstance().removeAll();
      _chassis.setForcedAutonFinish(false);
    }
  }

  /********************************************************************************************
   * Telop Mode
   ********************************************************************************************/
  /**
   * This function is called 1x when the robot is 1st enabled telop mode
   */
  @Override
  public void teleopInit() {
    _vision.set_isInVisionMode(false);
    Command changePipeline = new ChoosePipeline(LIMELIGHT_PIPELINE.CENTER_PNP);
    changePipeline.start();
    Command stopChassis = new StopChassis();
    stopChassis.start();
    _chassis.zeroSensors();
    _chassis.initiateRobotState();
    _chassis.setBrakeMode(NeutralMode.Brake);
        _scanTimeSamples = new MovingAverage(20);
    _dataLogger = GeneralUtilities.setupLogging("Teleop"); // init data logging
    _lastDashboardWriteTimeMSec = new Date().getTime(); // snapshot time to control spamming
    if(!_elevator.get_hasElevatorBeenZeroed()){
      Command zeroElevatorCommand = new ZeroElevatorEncoder();
      zeroElevatorCommand.start();
    }
    _chassis.setChassisState(ChassisState.PERCENT_VBUS);
    Command zeroClimber = new ZeroClimber();
    zeroClimber.start();

  
  }

   /* This function is called periodically during teleop mode.
   */
  @Override
  public void teleopPeriodic() {
    Scheduler.getInstance().run();  
    // _vision.turnOnLEDs();
    // Command drive = new DriveWithControllers(0.7, 0);
    // drive.start();
    //_chassis.updateChassis(Timer.getFPGATimestamp());

    //_vision.turnOnLEDs();
    // System.out.println(_elevator.getStoredTargetPosition());
  }

  /********************************************************************************************
   * Test Mode
   ********************************************************************************************/
  /**
   * This function is called 1x when the robot is 1st enabled test mode
   */
  @Override
  public void testInit() {}

  /**
   * This function is called periodically during test mode.
   */
  @Override
  public void testPeriodic() {}

  /********************************************************************************************
   * Disabled Mode
   ********************************************************************************************/
  /**
   * This function is called once each time the robot enters Disabled mode.
   * You can use it to reset any subsystem information you want to clear when
   * the robot is disabled.
   */
  @Override
  public void disabledInit() {
    _scanTimeSamples = new MovingAverage(20);
    _chassis.setBrakeMode(NeutralMode.Coast);
    _chassis.stop();
    Scheduler.getInstance().removeAll();
    _vision.turnOffLEDs();
  }

  /**
   * This function is called periodically during disabled mode.
   */
  @Override
  public void disabledPeriodic() {
    _vision.turnOffLEDs();

    SmartDashboard.putBoolean("Vision:IsPingable", _vision.get_isPingable());
  }
  
  /********************************************************************************************
   * General Support Methods
   ********************************************************************************************/
  /**
   * This function is called every robot packet, no matter the mode. Use
   * this for items like diagnostics that you want ran during disabled,
   * autonomous, teleoperated and test.
   *
   * <p>This runs after the mode specific periodic functions, but before
   * LiveWindow and SmartDashboard integrated updating.
   */
  @Override
  public void robotPeriodic() {
    // ============= Refresh Dashboard ============= 
    this.outputAllToDashboard();
    
    if(!isDisabled()) {
      // ============= Optionally Log Data =============
		  this.logAllData();
    }
  }

  /** Method to Push Data to ShuffleBoard */
  private void outputAllToDashboard() 
  {
	  // limit spamming
    //long scanCycleDeltaInMSecs = new Date().getTime() - _lastScanEndTimeInMSec;
    long scanCycleDeltaInMSecs = System.currentTimeMillis() - _lastScanEndTimeInMSec;    

    //System.out.println("DeltaT:" + scanCycleDeltaInMSecs);
    // add scan time sample to calc scan time rolling average
    _scanTimeSamples.add(new BigDecimal(scanCycleDeltaInMSecs));
    
    //if((new Date().getTime() - _lastDashboardWriteTimeMSec) > 100) {
    if((System.currentTimeMillis() - _lastDashboardWriteTimeMSec) > 20) 
    {
      // ----------------------------------------------
      // each subsystem should add a call to a outputToSmartDashboard method
      // to push its data out to the dashboard
      // ----------------------------------------------
      if(_chassis != null)              { _chassis.updateDashboard(); }
      if(_cargo != null)                { _cargo.updateDashboard(); }
      if(_climber != null)              { _climber.updateDashboard(); }
      if(_elevator != null)             { _elevator.updateDashboard(); }

      if(_autonChoosers != null)        { _autonChoosers.updateDashboard(); }
      if(_distanceRev2mSensor != null)  { _distanceRev2mSensor.updateDashboard(); }
      if(_vision != null)               { _vision.updateDashboard(); }
      if(_pressureSensor != null)       { _pressureSensor.updateDashboard(); }
      if(_navX != null)                 { _navX.updateDashboard(); }
      if(_cameraServer != null)         { _cameraServer.updateDashboard(); }
      if(_compressor != null)           { _compressor.updateDashboard(); }

      
      // write the overall robot dashboard info
      SmartDashboard.putString("Robot Build", _buildMsg);
      
      BigDecimal movingAvg = _scanTimeSamples.getAverage();
      DecimalFormat df = new DecimalFormat("####");
      SmartDashboard.putString("Scan Time (2 sec roll avg)", df.format(movingAvg) + " mSec");
      // snapshot last time
      //_lastDashboardWriteTimeMSec = new Date().getTime();
      _lastDashboardWriteTimeMSec = System.currentTimeMillis();
    }
    	// snapshot when this scan ended
      //_lastScanEndTimeInMSec = new Date().getTime();
      _lastScanEndTimeInMSec = System.currentTimeMillis();
    
	}

	/** Method for Logging Data to the USB Stick plugged into the RoboRio */
	private void logAllData() { 
		// always call this 1st to calc drive metrics
    	if(_dataLogger != null) {    	
	    	// create a new, empty logging class
        	LogDataBE logData = new LogDataBE();
        
        // ----------------------------------------------
        // ask each subsystem that exists to add its data
        // ----------------------------------------------
        if(_chassis != null)              { _chassis.updateLogData(logData); }
        if(_cargo != null)                { _cargo.updateLogData(logData); }
        if(_climber != null)              { _climber.updateLogData(logData); }
        if(_elevator != null)             { _elevator.updateLogData(logData); }

        if(_autonChoosers != null)        { _autonChoosers.updateLogData(logData); }
	    	if(_distanceRev2mSensor != null)  { _distanceRev2mSensor.updateLogData(logData); }
        if(_vision != null)               { _vision.updateLogData(logData); }
        if(_pressureSensor != null)       { _pressureSensor.updateLogData(logData); }
        if(_compressor != null)           { _compressor.updateLogData(logData); }
    
	    	_dataLogger.WriteDataLine(logData);
    	}
	}
}
