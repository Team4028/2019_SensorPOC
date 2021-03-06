/*----------------------------------------------------------------------------*/
/* Copyright (c) 2018 FIRST. All Rights Reserved.                             */
/* Open Source Software - may be modified and shared by FRC teams. The code   */
/* must be accompanied by the FIRST BSD license file in the root directory of */
/* the project.                                                               */
/*----------------------------------------------------------------------------*/

package frc.robot.sensors;

import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import frc.robot.interfaces.IDistanceSensor;
import frc.robot.sensors.revSrc.DistanceUnit;
import frc.robot.sensors.revSrc.VL53L0X;
import frc.robot.util.LogDataBE;
import frc.robot.RobotMap;
import frc.robot.util.GeneralUtilities;
/**
 * This class exposes the onboard REV 2M Distance Sensor 
 * 
 * Lead Student: Isabella
 */
public class DistanceRev2mSensor implements IDistanceSensor{

	private VL53L0X _distanceSensor;
	private double _distanceToTargetInInches;
	private boolean _didTimeoutOccur;
	private boolean _isSensorPresent;
	private final static double MAX_RANGE = 65;

    //=====================================================================================
	// Define Singleton Pattern
	//=====================================================================================
	private static DistanceRev2mSensor _instance = new DistanceRev2mSensor();
	
	public static DistanceRev2mSensor getInstance() {
		return _instance;
	}

	// private constructor for singleton pattern
	private DistanceRev2mSensor() {	
		_distanceSensor = new VL53L0X(RobotMap.I2C_SENSOR_PORT, 0x29);
		_isSensorPresent = _distanceSensor.doInitialize();
		setLongRangeMode();
		_distanceSensor.startContinuous();

		SmartDashboard.putBoolean("VL53LOX:isSensorPresent", get_isSensorPresent());

		// start a separate thread to talk to the sensor
		Thread t = new Thread(() -> {
			while (!Thread.interrupted()) {
				long start = System.nanoTime();
				_distanceToTargetInInches = _distanceSensor.getDistance(DistanceUnit.INCH);
				_didTimeoutOccur = _distanceSensor.didTimeoutOccur();
				long finish = System.nanoTime();
				long time_elapsed = finish-start;
				SmartDashboard.putNumber("distance sensor: thread time", time_elapsed/1000000);
				
			}
		});
		t.start();
	}

	private void setLongRangeMode() {
			_distanceSensor.setSignalRateLimit((float) 0.1);
			_distanceSensor.setVcselPulsePeriod(VL53L0X.vcselPeriodType.VcselPeriodPreRange, 18);
			_distanceSensor.setVcselPulsePeriod(VL53L0X.vcselPeriodType.VcselPeriodFinalRange, 14);
	}

    // ====================================================================
    // Property Accessors
    // ====================================================================

	@Override
	public double get_distanceToTargetInInches() {
		_distanceToTargetInInches = GeneralUtilities.roundDouble(_distanceToTargetInInches, 2);
		if(_distanceToTargetInInches > MAX_RANGE){
			_distanceToTargetInInches = -1;
		}
		return _distanceToTargetInInches;
	}

	public boolean get_didTimeoutOccur() {
		return _didTimeoutOccur;
	} 
	public boolean get_isSensorPresent() {
		return _isSensorPresent;
	} 

	// ====================================================================
    // Logging Methods
	// ====================================================================
	
	public void updateLogData(LogDataBE logData) {
	}

	public void updateDashboard(){
		SmartDashboard.putNumber("VL53LOX:DistanceInInches", get_distanceToTargetInInches());
		SmartDashboard.putBoolean("VL53LOX:didTimeoutOccur", get_didTimeoutOccur());
		
	}
}