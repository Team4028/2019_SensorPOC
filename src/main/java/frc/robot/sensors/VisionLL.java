/*----------------------------------------------------------------------------*/
/* Copyright (c) 2018 FIRST. All Rights Reserved.                             */
/* Open Source Software - may be modified and shared by FRC teams. The code   */
/* must be accompanied by the FIRST BSD license file in the root directory of */
/* the project.                                                               */
/*----------------------------------------------------------------------------*/

package frc.robot.sensors;

import edu.wpi.first.networktables.NetworkTableInstance;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import frc.robot.interfaces.IVisionSensor;
import frc.robot.util.LogDataBE;

/**
 * This class exposes the OnBoard LimeLite2 Vision sensor
 * 
 * Lead Student: Patrick Bruns
 */
public class VisionLL implements IVisionSensor {

    // =====================================================================================
    // Define Singleton Pattern
    // =====================================================================================
    private static VisionLL _instance = new VisionLL();

    public static VisionLL getInstance() {
        return _instance;
    }

    // private constructor for singleton pattern
    private VisionLL() {
    }

    @Override
    public double get_angle1InDegrees() {
        return Math.round(NetworkTableInstance.getDefault().getTable("limelight").getEntry("tx").getDouble(0));
    }

    @Override
    public double get_distanceToTargetInInches() {
        double heightofBoundedBox = NetworkTableInstance.getDefault().getTable("limelight").getEntry("tvert")
                .getDouble(0);
        double widthOfBoundedBox = NetworkTableInstance.getDefault().getTable("limelight").getEntry("thor")
                .getDouble(0);
        double areaofBoundedBox = heightofBoundedBox * widthOfBoundedBox;
        double distanceInIn = (1606.9 * Math.pow(areaofBoundedBox, -0.443));

        return distanceInIn;
    }

    @Override
    public boolean get_isTargetInFOV() {
        if (NetworkTableInstance.getDefault().getTable("limelight").getEntry("tv").getDouble(0) == 1) {
            return true;
        } else {
            return false;
        }
    }

    public void turnOffLimelightLEDs() {
        NetworkTableInstance.getDefault().getTable("limelight").getEntry("ledMode").setNumber(1);
    }

    public void turnOnLimelightLEDs() {
        NetworkTableInstance.getDefault().getTable("limelight").getEntry("ledMode").setNumber(3);
    }

    //=====================================================================================
	// Helper Methods
	//=====================================================================================  
	
    public void updateLogData(LogDataBE logData) {

    }

    public void updateDashboard() {
        SmartDashboard.putString("Vision:CameraType", "Limelight");
        SmartDashboard.putBoolean("Vision:IsTargetInFOV", get_isTargetInFOV());
        SmartDashboard.putNumber("Vision:Angle1InDegrees", get_angle1InDegrees());
        SmartDashboard.putNumber("Vision:DistanceInInches", get_distanceToTargetInInches());
    }
}
