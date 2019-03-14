package frc.robot;

import edu.wpi.first.wpilibj.DoubleSolenoid;
import edu.wpi.first.wpilibj.DoubleSolenoid.Value;

public class Constants {
	// Solenoid Positions
	public static final Value SHIFTER_LOW_GEAR_POS = DoubleSolenoid.Value.kReverse;
	public static final Value SHIFTER_HIGH_GEAR_POS = DoubleSolenoid.Value.kForward;

	// Path Following Constants
	public static final double MIN_LOOKAHEAD = 10.0; // inches
	public static final double MIN_LOOKAHEAD_SPEED = 9.0; // inches per second
	public static final double MAX_LOOKAHEAD = 20; // inches
	public static final double MAX_LOOKAHEAD_SPEED = 120.0; // inches per second
	public static final double DELTA_LOOKAHEAD = MAX_LOOKAHEAD - MIN_LOOKAHEAD;
	public static final double DELTA_LOOKAHEAD_SPEED = MAX_LOOKAHEAD_SPEED - MIN_LOOKAHEAD_SPEED;

	public static final double INERTIA_STEERING_GAIN = 0; // angular velocity command is multiplied by this gain *
							
	public static final double SEGMENT_COMPLETION_TOLERANCE = 0.1; // inches
	public static final double PATH_DEFAULT_ACCEL = 100.0; // inches per second^2
	public static final double PATH_DEFAULT_DECEL = 100.0;
	public static final double PATH_FOLLOWING_MAX_VEL = 120.0; // inches per second
	public static final double PATH_FOLLOWING_PROFILE_KP = 6.0; 
	public static final double PATH_FOLLOWING_PROFILE_KI = 0.01;	//0.03;
	public static final double PATH_FOLLOWING_PROFILE_KV = 0.02; //0.02;
	public static final double PATH_FOLLOWING_PROFILE_KFFV = 1.; //1. //.8
	public static final double PATH_FOLLOWING_PROFILE_KFFA = .1; //.1 //.05
	public static final double PATH_FOLLOWING_GOAL_POS_TOLERANCE = 0.75;
	public static final double PATH_FOLLOWING_GOAL_VEL_TOLERANCE = 12.0;
	public static final double PATH_STOP_STEERING_DISTANCE = 2.0;


	public static final double TRACK_SCRUBBING_FACTOR = 1.0;
	// Units: setpoint, error, and output are in inches per second. 
	public static final double DRIVE_VELOCITY_NOMINAL_OUTPUT = 0.05;
	public static final double DRIVE_VELOCITY_MAX_SETPOINT = 15 * 12.0; // 15 fps
	
	//Deadbands for Chassis
	public static double ENCODER_COUNTS_PER_WHEEL_REV = 4008;
	public static final double CHASSIS_DRIVE_SET_DISTANCE_DEADBAND = 1.0*ENCODER_COUNTS_PER_WHEEL_REV/(6*Math.PI);



	public static final int BIG_NUMBER = (int)1e6;
	public static final double EPSILON_NEGATIVE_6 = 1e-6;
    public static final double EPSILON_NEGATIVE_9 = 1e-9;

	/* Robot Physical Constants */
	// Wheels
	public static final double DRIVE_WHEEL_DIAMETER_IN = 6;
	public static final double TRACK_WIDTH_INCHES = 24.25;

}
