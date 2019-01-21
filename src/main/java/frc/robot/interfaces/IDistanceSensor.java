/*----------------------------------------------------------------------------*/
/* Copyright (c) 2018 FIRST. All Rights Reserved.                             */
/* Open Source Software - may be modified and shared by FRC teams. The code   */
/* must be accompanied by the FIRST BSD license file in the root directory of */
/* the project.                                                               */
/*----------------------------------------------------------------------------*/

package frc.robot.interfaces;

/**
 * This interface defines the methods all Distance Sensor classes must implement
 */
public interface IDistanceSensor {

    // returns a distance estimate (in inches) from the Robot to the target 
    // note this is only an approximation and is valid from 3ft -> 10 1ft
    public double get_distanceToTargetInInches();
}
