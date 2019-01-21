/*----------------------------------------------------------------------------*/
/* Copyright (c) 2018 FIRST. All Rights Reserved.                             */
/* Open Source Software - may be modified and shared by FRC teams. The code   */
/* must be accompanied by the FIRST BSD license file in the root directory of */
/* the project.                                                               */
/*----------------------------------------------------------------------------*/

package frc.robot.ux;

import edu.wpi.first.wpilibj.Spark;

/**
 * This class controls the onboard LEDs Lead Student: Parker Johnson
 */
public class LEDController 
{
	private Spark LEDstrip;
	public boolean _areledsorange;
    public boolean _areledsblinking;
    private static final double REDZONE = 27.0; 
	private static final double YELLOWZONE = 10.0;
	private static final double GREENZONE = 2.0;

   
    //=====================================================================================
	// Define Singleton Pattern
	//=====================================================================================
	private static LEDController _instance = new LEDController();
	
	public static LEDController getInstance() 
	{
		return _instance;
	}
	
	// private constructor for singleton pattern
	private LEDController() 
	{	
		LEDstrip = new Spark(0);
	}
	
	// call this method to display the correct LED Color
	public void set_targetangle (double currentAngleInDegrees)
	{
		if(Math.abs(currentAngleInDegrees) > REDZONE)
        {
            whiteLights();
		}
		else if(Math.abs(currentAngleInDegrees) >= YELLOWZONE && Math.abs(currentAngleInDegrees)<= REDZONE)
        {
            redLEDstrip();
		}
		else if(Math.abs(currentAngleInDegrees) >= GREENZONE && Math.abs(currentAngleInDegrees)<= YELLOWZONE)
		{
			orangeLEDstrip();
		}
		else if(Math.abs(currentAngleInDegrees) <= YELLOWZONE)
		{
			partyMode();
		}     
	}
	private void partyMode()
	{
		LEDstrip.set(-0.99);
	}
	private void whiteLights()
	{
		LEDstrip.set(0.99);
	}
	private void greenLights()
    {
        LEDstrip.set(0.75);
        _areledsblinking = true;
    }

    private void orangeLEDstrip()
    {
        LEDstrip.set(0.67);
        _areledsorange = true;
    }
    private void redLEDstrip()
    {
        LEDstrip.set(0.61);
    }
    private void fireLEDS()
    {
        LEDstrip.set(0.57);
        _areledsblinking = false;
        
    }
    private void blueLEDs()
    {
        LEDstrip.set(-0.95);
        _areledsorange = false;
	}
	
	
    /*public void redZoneCheck()
    {
        if(getCurrentDistance() >= YELLOWZONE && getCurrentDistance() <= REDZONE)
        {
            orangeLEDstrip();
        }
        else
        {
            negRedZoneCheck();
            _areledsorange = false;
        }
    }
    public void negRedZoneCheck()
    {
        if(getCurrentDistance() <= NEGREDZONE && getCurrentDistance() >= NEGYEllOWZONE)
        {
            orangeLEDstrip();
        }
        else
        {
            redLEDstrip();
            _areledsorange = false;
        }
    }
    public void isTargetLocked()
    {
        if(getCurrentDistance() <= YELLOWZONE && getCurrentDistance() >= NEGYEllOWZONE)
        {
            strobegreenLights();
        }
        else
        {
            redZoneCheck();
        }
    }*/
}
