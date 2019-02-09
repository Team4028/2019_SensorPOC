/*----------------------------------------------------------------------------*/
/* Copyright (c) 2018 FIRST. All Rights Reserved.                             */
/* Open Source Software - may be modified and shared by FRC teams. The code   */
/* must be accompanied by the FIRST BSD license file in the root directory of */
/* the project.                                                               */
/*----------------------------------------------------------------------------*/

package frc.robot.sensors;

import edu.wpi.first.wpilibj.AnalogInput;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import frc.robot.RobotMap;
import frc.robot.interfaces.IBeakSquadSubsystem;
import frc.robot.util.LogDataBE;

/**
 * This class exposes the OnBoard Analog Pressure Sensor 
 * 
 * Lead Student: Peter
 */
public class StoredPressureSensor implements IBeakSquadSubsystem {

	private double SUPPLY_VOLTAGE = 4.8;
	private AnalogInput _analogPressureSensor;

	// =====================================================================================
	// Define Singleton Pattern
	// =====================================================================================
	private static StoredPressureSensor _instance = new StoredPressureSensor();

	public static StoredPressureSensor getInstance() {
		return _instance;
	}

	// private constructor for singleton pattern
	private StoredPressureSensor() {
		_analogPressureSensor = new AnalogInput(RobotMap.AIN_STORED_PRESSURE_SENSOR_PORT);
	}

	public double get_storedPSI() {
		return 250 * (_analogPressureSensor.getVoltage() / SUPPLY_VOLTAGE) - 25;
	}

	// ====================================================================
    // Logging Methods
	// ====================================================================
	@Override
	public void updateLogData(LogDataBE logData) {

	}

	@Override
	public void updateDashboard() {
		SmartDashboard.putNumber("StoredPressureSensor:PressureInPSI", get_storedPSI());
	}
}
