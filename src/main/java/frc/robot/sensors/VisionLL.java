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

    public enum LIMELIGHT_PIPELINE {
        RIGHT,
        CENTER,
        LEFT,
        CENTER_PNP;
    }

    private double HORIZONAL_CAMERA_OFFSET_IN = 6;
    private double VERTICAL_CAMERA_OFFSET_IN = 15;
    private boolean _isInVisionMode = false;

    private GyroNavX _navX = GyroNavX.getInstance();
    // =====================================================================================
    // Define Singleton Pattern
    // =====================================================================================
    private static VisionLL _instance = new VisionLL();

    public static VisionLL getInstance() {
        return _instance;
    }

    // private constructor for singleton pattern
    private VisionLL() {
        NetworkTableInstance.getDefault().getTable("limelight").getEntry("pipeline").setNumber(1);
    }

    @Override
    public double get_angle1InDegrees() {
        return Math.round(NetworkTableInstance.getDefault().getTable("limelight").getEntry("tx").getDouble(0));
    }

    public double getTheta(){
        return NetworkTableInstance.getDefault().getTable("limelight").getEntry("tx").getDouble(0);
    }

    public double getPhi(){
        return NetworkTableInstance.getDefault().getTable("limelight").getEntry("tz").getDouble(0);
    }

    //@Override
   /* public double get_distanceToTargetInInches() {
        double heightofBoundedBox = NetworkTableInstance.getDefault().getTable("limelight").getEntry("tvert")
                .getDouble(0);
        double widthOfBoundedBox = NetworkTableInstance.getDefault().getTable("limelight").getEntry("thor")
                .getDouble(0);
        double areaofBoundedBox = heightofBoundedBox * widthOfBoundedBox;
        double distanceInIn = (1606.9 * Math.pow(areaofBoundedBox, -0.443));

        return distanceInIn;
    } */

    @Override
    public double get_distanceToTargetInInches() {
        return 0;
    }

    @Override
    public boolean get_isTargetInFOV() {
        if (NetworkTableInstance.getDefault().getTable("limelight").getEntry("tv").getDouble(0) == 1) {
            return true;
        } else {
            return false;
        }
    }

    public double get_xOffset() {
        double[] defaultValue = new double[6];
        double[] camtran = NetworkTableInstance.getDefault().getTable("limelight").getEntry("camtran").getDoubleArray(defaultValue);
        double xOffset = camtran[0];
        return xOffset;
    }

    public double get_revisedDistance(){
        double actualDistance = Math.sqrt(Math.pow(get_distanceToTargetInInches(), 2)
        // - Math.pow(HORIZONAL_CAMERA_OFFSET_IN/Math.cos(Math.abs(_navX.)), 2)
         - Math.pow(VERTICAL_CAMERA_OFFSET_IN, 2));
        return actualDistance;
    }

    public void changeLimelightPipeline(LIMELIGHT_PIPELINE pipeline) {
        switch(pipeline){
            case RIGHT:
                NetworkTableInstance.getDefault().getTable("limelight").getEntry("pipeline").setNumber(0);
                break;
            case CENTER:
                NetworkTableInstance.getDefault().getTable("limelight").getEntry("pipeline").setNumber(1);
                break;
            case LEFT:
                NetworkTableInstance.getDefault().getTable("limelight").getEntry("pipeline").setNumber(2);
                break;
            case CENTER_PNP:
                NetworkTableInstance.getDefault().getTable("limelight").getEntry("pipeline").setNumber(3);
                break;
        }
    }

    @Override
    public void turnOnLEDs() {
        NetworkTableInstance.getDefault().getTable("limelight").getEntry("ledMode").setNumber(3);
    }

    @Override
    public void turnOffLEDs() {
        NetworkTableInstance.getDefault().getTable("limelight").getEntry("ledMode").setNumber(1);
    }

    public boolean isInVisionMode() {
        return _isInVisionMode;
    }

    @Override
    public void set_isInVisionMode(boolean isInVisionMode) {
        _isInVisionMode = isInVisionMode;
        if(isInVisionMode){
            turnOnLEDs();
        } else {
            turnOffLEDs();
        }
    }
    
    public double getTrueDistance()
    {
        if(DistanceRev2mSensor.getInstance().get_distanceToTargetInInches()>0)
        {
            return DistanceRev2mSensor.getInstance().get_distanceToTargetInInches();
        }
        else if(this.get_isTargetInFOV())
        {
            return get_revisedDistance();
        }
        else 
        {
            return Double.NaN;
        }
    }

    //=====================================================================================
	// Helper Methods
	//=====================================================================================  
    
    @Override
    public void updateLogData(LogDataBE logData) {}

    @Override
    public void updateDashboard() {
        SmartDashboard.putNumber("Distance to Target Inches", getTrueDistance());
        SmartDashboard.putString("Vision:CameraType", "Limelight");
        SmartDashboard.putBoolean("Vision:IsTargetInFOV", get_isTargetInFOV());
        SmartDashboard.putNumber("Vision:Angle1InDegrees", get_angle1InDegrees());
        SmartDashboard.putNumber("Vision:DistanceInInches", get_distanceToTargetInInches());
        SmartDashboard.putNumber("Vision:ActualDistance", get_revisedDistance());
        SmartDashboard.putNumber("Vision:XOffset", get_xOffset());
    }
}