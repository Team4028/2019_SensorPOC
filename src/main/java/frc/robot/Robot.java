/*----------------------------------------------------------------------------*/
/* Copyright (c) 2018 FIRST. All Rights Reserved.                             */
/* Open Source Software - may be modified and shared by FRC teams. The code   */
/* must be accompanied by the FIRST BSD license file in the root directory of */
/* the project.                                                               */
/*----------------------------------------------------------------------------*/

package frc.robot;

import java.math.BigDecimal;
import java.text.DecimalFormat;
import java.util.Date;

import edu.wpi.first.wpilibj.TimedRobot;
import edu.wpi.first.wpilibj.command.Scheduler;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;

import frc.robot.sensors.VisionLL;

import frc.robot.sensors.DistanceRev2mSensor;

import frc.robot.subsystems.Chassis;
import frc.robot.util.DataLogger;
import frc.robot.util.GeneralUtilities;
import frc.robot.util.LogDataBE;
import frc.robot.util.MovingAverage;
import frc.robot.ux.AutonChoosers;
import frc.robot.ux.LEDController;

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
  private AutonChoosers _autonChoosers = AutonChoosers.getInstance();

  private Chassis _chassis = Chassis.getInstance();
  private LEDController _leds = LEDController.getInstance();
  private VisionLL _vision = VisionLL.getInstance();



  private DistanceRev2mSensor _DistanceRev2mSensor = DistanceRev2mSensor.getInstance();

	// class level working variables
	private DataLogger _dataLogger = null;
	private String _buildMsg = "?";
 	long _lastScanEndTimeInMSec;
 	long _lastDashboardWriteTimeMSec;
	MovingAverage _scanTimeSamples;
	public double _startTime;

  /********************************************************************************************
   * This function is run when the robot is first started up and should be used
   * for any initialization code.
   ********************************************************************************************/
  @Override
  public void robotInit() {
    _buildMsg = GeneralUtilities.WriteBuildInfoToDashboard(ROBOT_NAME);
  }

  /********************************************************************************************
   * Autonomous Mode
   ********************************************************************************************/
  /**
   * This function is called 1x when the robot is 1st enabled autonomous mode
   */
  @Override
  public void autonomousInit() {
    _scanTimeSamples = new MovingAverage(20);
    _lastDashboardWriteTimeMSec = new Date().getTime(); // snapshot time to control spamming
    _dataLogger = GeneralUtilities.setupLogging("Auton"); // init data logging
    
  }

  /**
   * This function is called periodically during autonomous mode.
   */
  @Override
  public void autonomousPeriodic() {
    Scheduler.getInstance().run();
    _leds.set_targetangle(_vision.get_angle1InDegrees(), _vision.canLLSeeTarget(), _DistanceRev2mSensor.get_distanceToTargetInInches());
       System.out.println(_vision.canLLSeeTarget());
    
  }

  /********************************************************************************************
   * Telop Mode
   ********************************************************************************************/
  /**
   * This function is called 1x when the robot is 1st enabled telop mode
   */
  @Override
  public void teleopInit() {
    _scanTimeSamples = new MovingAverage(20);
    _dataLogger = GeneralUtilities.setupLogging("Teleop"); // init data logging
		_lastDashboardWriteTimeMSec = new Date().getTime(); // snapshot time to control spamming
  }

  /**
   * This function is called periodically during teleop mode.
   */
  @Override
  public void teleopPeriodic() {
    Scheduler.getInstance().run();
  }

  /********************************************************************************************
   * Test Mode
   ********************************************************************************************/
  /**
   * This function is called 1x when the robot is 1st enabled test mode
   */
  @Override
  public void testInit() {
  }

  /**
   * This function is called periodically during test mode.
   */
  @Override
  public void testPeriodic() {
  }

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
  }

  /**
   * This function is called periodically during disabled mode.
   */
  @Override
  public void disabledPeriodic() {
    Scheduler.getInstance().run();
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
    
    if(!isDisabled())
    {
      // ============= Optionally Log Data =============
		  this.logAllData();
    }
  }

  /** Method to Push Data to ShuffleBoard */
	private void outputAllToDashboard() {
		// limit spamming
    	long scanCycleDeltaInMSecs = new Date().getTime() - _lastScanEndTimeInMSec;
    	// add scan time sample to calc scan time rolling average
    	_scanTimeSamples.add(new BigDecimal(scanCycleDeltaInMSecs));
    	
    	if((new Date().getTime() - _lastDashboardWriteTimeMSec) > 100) {

        // ----------------------------------------------
    		// each subsystem should add a call to a outputToSmartDashboard method
    		// to push its data out to the dashboard
        // ----------------------------------------------
        _autonChoosers.updateDashboard();
        _chassis.updateDashboard(); 
        _DistanceRev2mSensor.updateDashboard();
	    	
    		// write the overall robot dashboard info
	    	SmartDashboard.putString("Robot Build", _buildMsg);
	    	
	    	BigDecimal movingAvg = _scanTimeSamples.getAverage();
	    	DecimalFormat df = new DecimalFormat("####");
	    	SmartDashboard.putString("Scan Time (2 sec roll avg)", df.format(movingAvg) + " mSec");
    		// snapshot last time
    		_lastDashboardWriteTimeMSec = new Date().getTime();
    	}
    	
    	// snapshot when this scan ended
    	_lastScanEndTimeInMSec = new Date().getTime();
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
        _autonChoosers.updateLogData(logData);
	    	_chassis.updateLogData(logData);
        _DistanceRev2mSensor.updateLogData(logData);
        
	    	_dataLogger.WriteDataLine(logData);
    	}
	}
}
