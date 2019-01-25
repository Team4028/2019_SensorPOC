/*----------------------------------------------------------------------------*/
/* Copyright (c) 2018 FIRST. All Rights Reserved.                             */
/* Open Source Software - may be modified and shared by FRC teams. The code   */
/* must be accompanied by the FIRST BSD license file in the root directory of */
/* the project.                                                               */
/*----------------------------------------------------------------------------*/

package frc.robot.sensors;

import edu.wpi.first.wpilibj.AnalogInput;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import frc.robot.interfaces.IBeakSquadSubsystem;
import frc.robot.util.LogDataBE;
import frc.robot.RobotMap;

/**
 * Add your docs here.
 */
public class PressureSensor implements IBeakSquadSubsystem 
{
    private double SUPPLY_VOLTAGE = 4.8;
    private AnalogInput _analogPressureSensor;
    
    private PressureSensor()
    {
        _analogPressureSensor = new AnalogInput(RobotMap.STORED_PRESSURE_SENSOR_AIO_PORT);
    }

   private static PressureSensor _instance = new PressureSensor();
   public static PressureSensor getInstance()
   {
       return _instance;
   }

    public double get_storedPressure() 
    {
        return 250*(_analogPressureSensor.getVoltage()/SUPPLY_VOLTAGE)-25;  
    }

    @Override
    public void updateLogData(LogDataBE logData) 
    {

    }

    @Override
    public void updateDashboard() 
    {
        SmartDashboard.putNumber("Pressure Sensor: Pressure in PSI", get_storedPressure());
        //SmartDashboard.putNumber("Pressure Sensor: Output Voltage", _analogPressureSensor.getVoltage());
    }
}
