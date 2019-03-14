/*----------------------------------------------------------------------------*/
/* Copyright (c) 2018 FIRST. All Rights Reserved.                             */
/* Open Source Software - may be modified and shared by FRC teams. The code   */
/* must be accompanied by the FIRST BSD license file in the root directory of */
/* the project.                                                               */
/*----------------------------------------------------------------------------*/

package frc.robot.sensors;

import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import frc.robot.interfaces.IDistanceSensor;
import frc.robot.interfaces.IVisionSensor;
import frc.robot.util.LogDataBE;

/**
 * Add your docs here.
 */
public class FusedDistanceSenor implements IDistanceSensor{

    private DistanceRev2mSensor _distanceRev2mSensor = DistanceRev2mSensor.getInstance(); 
    private IVisionSensor _vision = VisionLL.getInstance();      // Limelight
    //private IVisionSensor _vision = VisionIP.getInstance();   // IPhone

    private double _distanceInInches = -1.0;
    private String _distanceSensor = "???";

    // =====================================================================================
    // Define Singleton Pattern
    // =====================================================================================
    private static FusedDistanceSenor _instance = new FusedDistanceSenor();

    public static FusedDistanceSenor getInstance() {
        return _instance;
    }

    // private constructor for singleton pattern
    private FusedDistanceSenor() {
    }

    @Override
    public double get_distanceToTargetInInches() {
        _distanceInInches = -1.0;
        _distanceSensor = "???";

        // step 1: do we have valid distance data from REV Sensor
        double revDistance = _distanceRev2mSensor.get_distanceToTargetInInches();
        boolean isTargetInFOV = _vision.get_isTargetInFOV();
        double visionDistance = _vision.get_distanceToTargetInInches();

        if(revDistance > 0)
        {
            _distanceInInches = revDistance;
            _distanceSensor = "Rev2M";
        }
        else if(isTargetInFOV)
        {
            _distanceInInches = _vision.get_distanceToTargetInInches();
            _distanceSensor = "Vision";
        }

        return _distanceInInches;
    }

    // ====================================================================
    // Logging Methods
	// ====================================================================
	
	public void updateLogData(LogDataBE logData) {
	}

	public void updateDashboard(){
		SmartDashboard.putNumber("Distance:DistanceInInches", _distanceInInches);
		SmartDashboard.putString("Distance:Sensor", _distanceSensor);
		
	}
}
