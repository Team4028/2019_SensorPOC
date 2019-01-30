/*----------------------------------------------------------------------------*/
/* Copyright (c) 2018 FIRST. All Rights Reserved.                             */
/* Open Source Software - may be modified and shared by FRC teams. The code   */
/* must be accompanied by the FIRST BSD license file in the root directory of */
/* the project.                                                               */
/*----------------------------------------------------------------------------*/

package frc.robot.auton.pathfollowing;

import frc.robot.sensors.GyroNavX;
import frc.robot.sensors.VisionLL;
import edu.wpi.first.wpilibj.DriverStation;
import frc.robot.RobotMap;

/**
 * This class exposes the OnBoard Navigation Sensor Lead Student:
 */
public class chassisNavX {
    //=====================================================================================
	// Define Singleton Pattern
	//=====================================================================================
	// define class level working variables
	
    private static chassisNavX _instance = new chassisNavX();
    private VisionLL _visionLL = VisionLL.getInstance();
    private static GyroNavX _navX = GyroNavX.getInstance();
    private double offset = 0;
	
	public static chassisNavX getInstance() {
		return _instance;
	}
	
	// private constructor for singleton pattern
	private chassisNavX() {	
    }
    
    public double getOffset(){
        return offset;
    }

    public void setOffset(double os){
        offset = os;
    }

    public void reAxizeNow(){
        setOffset(_navX.getYaw());
    }

	public double get_angle2InDegreesFromLL() {
		double angle2 =-90 - _visionLL.get_angle1InDegrees() - _navX.getYaw();
		return angle2;
	}
	
    public double getYaw() { 
	return _navX.getYaw() - offset;
	}
	
	public void zeroYaw() { 
        _navX.zeroYaw();        
	}
	
	public double getPitch() { //Axis Perpendicular to the Front/Back of the robot
	return _navX.getPitch();
	}
}
