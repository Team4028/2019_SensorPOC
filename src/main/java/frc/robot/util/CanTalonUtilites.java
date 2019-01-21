/*----------------------------------------------------------------------------*/
/* Copyright (c) 2018 FIRST. All Rights Reserved.                             */
/* Open Source Software - may be modified and shared by FRC teams. The code   */
/* must be accompanied by the FIRST BSD license file in the root directory of */
/* the project.                                                               */
/*----------------------------------------------------------------------------*/

package frc.robot.util;

import com.ctre.phoenix.motorcontrol.can.TalonSRX;

/**
 * Add your docs here.
 */
public class CanTalonUtilites {

    public static void setPIDFGains(TalonSRX talon, double[] gains) {
    	talon.config_kP(0, gains[0], 0);
		talon.config_kI(0, gains[1], 0);
		talon.config_kD(0, gains[2], 0);
		talon.config_kF(0, gains[3], 0);
    }
    
    public static void setMotionMagicConstants(TalonSRX talon, int[] constants) {
    	talon.configMotionCruiseVelocity(constants[0], 0);
    	talon.configMotionAcceleration(constants[1], 0);
    }
}
