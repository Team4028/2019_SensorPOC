/*----------------------------------------------------------------------------*/
/* Copyright (c) 2018 FIRST. All Rights Reserved.                             */
/* Open Source Software - may be modified and shared by FRC teams. The code   */
/* must be accompanied by the FIRST BSD license file in the root directory of */
/* the project.                                                               */
/*----------------------------------------------------------------------------*/

package frc.robot.interfaces;

/**
 * This interface defines the methods all Vision classes must implement
 */
public interface IVisionSensor extends IDistanceSensor, IBeakSquadSubsystem{

    // returns the angle (in degrees) from the robot centerline directly to the target centerline
    public double get_angle1InDegrees();

    // returns true if a target is in the Field Of View (FOV)
    public boolean get_isTargetInFOV();

    public void turnOnLEDs();

    public void turnOffLEDs();

    public void set_isInVisionMode(boolean isInVisionMode);
}
