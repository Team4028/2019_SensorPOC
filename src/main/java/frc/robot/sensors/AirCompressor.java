/*----------------------------------------------------------------------------*/
/* Copyright (c) 2018 FIRST. All Rights Reserved.                             */
/* Open Source Software - may be modified and shared by FRC teams. The code   */
/* must be accompanied by the FIRST BSD license file in the root directory of */
/* the project.                                                               */
/*----------------------------------------------------------------------------*/

package frc.robot.sensors;

import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import frc.robot.RobotMap;
import frc.robot.interfaces.IBeakSquadSubsystem;
import frc.robot.util.LogDataBE;
import edu.wpi.first.wpilibj.Compressor;


/**
 * This class exposes the compressor 
 * 
 * Lead Student: Isabella
 */
public class AirCompressor implements IBeakSquadSubsystem {

    private Compressor _compressor;
    private int _numberCycles = 0;

    //=====================================================================================
	// Define Singleton Pattern
	//=====================================================================================
    private static AirCompressor _instance = new AirCompressor();

    public static AirCompressor get_instance() {
        return _instance;
    }

    // private constructor for singleton pattern
    private AirCompressor() {
        _compressor = new Compressor(RobotMap.PCM_CAN_ADDR);
    }

    // ====================================================================
    // Logging Methods
	// ====================================================================
    public void updateDashboard(){
        boolean isCompressorRunning = _compressor.enabled();
        if(isCompressorRunning == true){
            _numberCycles++;
        }
        //SmartDashboard.putBoolean("compressor:isCompressorEnabled", isCompressorRunning);
        SmartDashboard.putNumber("AirCompressor:RunningTimeInSec", _numberCycles*.02);
    }

    public void updateLogData(LogDataBE logData) {
	}

}
