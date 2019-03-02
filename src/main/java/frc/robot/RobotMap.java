package frc.robot;

import edu.wpi.first.wpilibj.I2C;
import edu.wpi.first.wpilibj.SPI;

/**
 * The RobotMap is a mapping from the ports sensors and actuators are wired into
 * to a variable name. This provides flexibility changing wiring, makes checking
 * the wiring easier and significantly reduces the number of magic numbers
 * floating around.
 */
public class RobotMap {
	// Drivers Station Gamepad USB Ports
	public static final int DRIVER_GAMEPAD_USB_PORT = 0;
	public static final int OPERATOR_GAMEPAD_USB_PORT = 1;
	public static final int ENGINEERING_GAMEPAD_USB_PORT = 2;
	public static final int ENGINEERING_GAMEPAD_B_USB_PORT = 3;
	
	// PCM CAN Bus Address
	public static final int PCM_CAN_ADDR = 0;
	
	// Motor Controller Can Bus Address
	public static final int LEFT_DRIVE_MASTER_CAN_ADDR = 1;
	public static final int LEFT_DRIVE_SLAVE_CAN_ADDR = 2;
	public static final int RIGHT_DRIVE_MASTER_CAN_ADDR = 3;
	public static final int RIGHT_DRIVE_SLAVE_CAN_ADDR = 4;
	public static final int CLIMBER_LIFT_CAN_ADDR = 5;
	public static final int CLIMBER_DRIVE_CAN_ADDR = 6;
	public static final int ELEVATOR_SLAVE_CAN_ADDR = 7;
	public static final int ELEVATOR_MASTER_CAN_ADDR = 8;
	public static final int INFEED_DRIVE_CAN_ADDR = 9;

	// DIO Ports
	public static final int CARGO_LIMIT_SWITCH_DIO_PORT = 9;
	// Analog Ports
	public static final int AIN_STORED_PRESSURE_SENSOR_PORT = 0;
	
	// NavX (on Roborio)
	public static final SPI.Port NAVX_PORT = SPI.Port.kMXP;
	
	// PWM Ports on RoboRIO
	public static final int PWM_LED_PORT = 0;
	
	// PCM Ports
	public static final int PCM_FORWARD_BEAK_SOLENOID_PORT = 6;
	public static final int PCM_REVERSE_BEAK_SOLENOID_PORT = 7;
	public static final int PCM_FORWARD_PUNCH_SOLENOID_PORT = 2;
	public static final int PCM_REVERSE_PUNCH_SOLENOID_PORT = 3;
	public static final int PCM_FORWARD_INOUT_SOLENOID_PORT = 4;
	public static final int PCM_REVERSE_INOUT_SOLENOID_PORT = 5;
	public static final int PCM_REVERSE_BUCKET_SOLENOID_PORT = 0;
	public static final int PCM_FORWARD_BUCKET_SOLENOID_PORT = 1;

	//I2C Ports
	public static final I2C.Port I2C_SENSOR_PORT = I2C.Port.kOnboard;

	//socket client
	public static final String SOCKET_CLIENT_CONNECTION_IPADRESS = "10.40.28.6";
	public static final int SOCKET_CLIENT_CONNECTION_PORT = 1337;

	//Camera Addresses
	public static final String RASPBERRY_PI_CAMERA_1_ADDRESS = "http://10.40.28.13:1181/stream.mjpg";
	public static final String RASPBERRY_PI_CAMERA_2_ADDRESS = "http://10.40.28.13:1182/stream.mjpg";
	public static final String LIMELIGHT_CAMERA_ADDRESS = "http://10.40.28.11:5800/";

	// Logging
	// this is where the USB stick is mounted on the RoboRIO filesystem.  
	// You can confirm by logging into the RoboRIO using WinSCP
	public static final String PRIMARY_LOG_FILE_PATH = "/media/sda1/logging";
    public static final String ALTERNATE_LOG_FILE_PATH = "/media/sdb1/logging";
}
