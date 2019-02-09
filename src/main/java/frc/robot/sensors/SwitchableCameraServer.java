/*----------------------------------------------------------------------------*/
/* Copyright (c) 2018 FIRST. All Rights Reserved.                             */
/* Open Source Software - may be modified and shared by FRC teams. The code   */
/* must be accompanied by the FIRST BSD license file in the root directory of */
/* the project.                                                               */
/*----------------------------------------------------------------------------*/

package frc.robot.sensors;

import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import frc.robot.RobotMap;

/**
 * Add your docs here.
 */
public class SwitchableCameraServer {

    private String _currentCameraAddress = RobotMap.RASPBERRY_PI_CAMERA_1_ADDRESS;
    private double _currentCameraIndex = 0;

    private static SwitchableCameraServer _instance = new SwitchableCameraServer();
	
	public static SwitchableCameraServer getInstance() {
		return _instance;
	}
	
	// private constructor for singleton pattern
	private SwitchableCameraServer() {	
		
    }

    public void switchCamera() {
        if(_currentCameraIndex == 0){
            _currentCameraIndex = 1;
            _currentCameraAddress = RobotMap.RASPBERRY_PI_CAMERA_1_ADDRESS;
        }
        else if (_currentCameraIndex == 1) {
            _currentCameraIndex = 2;
            _currentCameraAddress = RobotMap.RASPBERRY_PI_CAMERA_2_ADDRESS;
        }
        else if (_currentCameraIndex == 2) {
            _currentCameraIndex = 0;
            _currentCameraAddress = RobotMap.LIMELIGHT_CAMERA_ADDRESS;
            
        } else {
            DriverStation.reportWarning("Camera Index Out of Range", false);
            _currentCameraIndex = 0;
            switchCamera();
        }
        DriverStation.reportWarning("Camera Adress Set to:" + _currentCameraAddress, false);
    }

    public void displayLimelight() {
        _currentCameraAddress = RobotMap.LIMELIGHT_CAMERA_ADDRESS;
    }
    
    //=====================================================================================
	// Helper Methods
	//=====================================================================================  
	public void updateDashboard() {
		SmartDashboard.putString("CurrentCameraAddress", _currentCameraAddress);
	}
}
