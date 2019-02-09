/*----------------------------------------------------------------------------*/
/* Copyright (c) 2018 FIRST. All Rights Reserved.                             */
/* Open Source Software - may be modified and shared by FRC teams. The code   */
/* must be accompanied by the FIRST BSD license file in the root directory of */
/* the project.                                                               */
/*----------------------------------------------------------------------------*/

package frc.robot;

import edu.wpi.first.wpilibj.I2C;
import edu.wpi.first.wpilibj.SPI;

/**
 * The RobotMap is a mapping from the ports sensors and actuators are wired into
 * to a variable name. This provides flexibility changing wiring, makes checking
 * the wiring easier and significantly reduces the number of magic numbers
 * floating around.
 */
public class RobotMap 
{
	// Drivers Station Gamepad USB Ports
	public static final int DRIVER_GAMEPAD_USB_PORT = 0;
	public static final int OPERATOR_GAMEPAD_USB_PORT = 1;
	public static final int ENGINEERING_GAMEPAD_USB_PORT = 2;
	public static final int ENGINEERING_GAMEPAD_B_USB_PORT = 3;
	
	// PCM Can Bus Address
	public static final int PCM_CAN_ADDR = 0;
	public static final int CARGO_VICTOR_ADDR = 1;
	
	// Motor Controller Can Bus Address
	//public static final int LEFT_DRIVE_MASTER_CAN_ADDR = 0;
	//public static final int LEFT_DRIVE_SLAVE_CAN_ADDR = 0;
	//public static final int RIGHT_DRIVE_MASTER_CAN_ADDR = 0;
	//public static final int RIGHT_DRIVE_SLAVE_CAN_ADDR = 0;

	public static final int ELEVATOR_MASTER_CAN_ADDR = 14;
	public static final int ELEVATOR_SLAVE_CAN_ADDR = 0;

	//public static final int INFEED_CAN_ADDR = 0;

	//public static final int CLIMBER_LIFT_CAN_ADDR = 0;
	//public static final int CLIMBER_DRIVE_CAN_ADDR = 0;

	// DIO Ports
	
	// Analog Ports
	public static final int AIN_STORED_PRESSURE_SENSOR_PORT = 0;
	
	// NavX (on Roborio)
	public static final SPI.Port NAVX_PORT = SPI.Port.kMXP;
	
	
	// PWM Ports on RoboRIO
	public static final int PWM_LED_PORT = 0;
	
	// PCM Ports
	public static final int PCM_FORWARD_BEAK_SOLENOID_PORT = 0;
	public static final int PCM_REVERSE_BEAK_SOLENOID_PORT = 1;
	public static final int PCM_FORWARD_PUNCH_SOLENOID_PORT = 2;
	public static final int PCM_REVERSE_PUNCH_SOLENOID_PORT = 3;
	public static final int PCM_FORWARD_INOUT_SOLENOID_PORT = 4;
	public static final int PCM_REVERSE_INOUT_SOLENOID_PORT = 5;




	//I2C Ports
	public static final I2C.Port I2C_SENSOR_PORT = I2C.Port.kOnboard;

	//socket client
	public static final String SOCKET_CLIENT_CONNECTION_IPADRESS = "10.40.28.6";
	public static final int SOCKET_CLIENT_CONNECTION_PORT = 1337;

}
