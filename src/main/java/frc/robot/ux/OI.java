/*----------------------------------------------------------------------------*/
/* Copyright (c) 2018 FIRST. All Rights Reserved.                             */
/* Open Source Software - may be modified and shared by FRC teams. The code   */
/* must be accompanied by the FIRST BSD license file in the root directory of */
/* the project.                                                               */
/*----------------------------------------------------------------------------*/

package frc.robot.ux;

import frc.robot.RobotMap;
import frc.robot.commands.Climber.LiftClimber;
import frc.robot.commands.Climber.DriveClimber;
import frc.robot.commands.Climber.PositionClimber;
import frc.robot.commands.chassis.DriveWithControllers;
import frc.robot.commands.infeed.AquireHatch;
import frc.robot.commands.infeed.RunInfeedMotor;
import frc.robot.commands.infeed.ScoreHatch;
import frc.robot.commands.infeed.ToggleBeakPosition;
import frc.robot.commands.infeed.TogglePunch;
import frc.robot.commands.infeed.ToggleStartPos;
import frc.robot.util.BeakXboxController;

/**
 * This class interfaces with the Driver/Operator Station 
 * 	Lead Student:
 */
public class OI {
    private BeakXboxController _driverController;
    private BeakXboxController _operatorController;
    
    //=====================================================================================
	// Define Singleton Pattern
	//=====================================================================================
	private static OI _instance = new OI();
	
	public static OI getInstance() {
		return _instance;
	}
	
	// private constructor for singleton pattern
	private OI() 
	{	
		// =========== Driver ======================================
		_driverController = new BeakXboxController(RobotMap.DRIVER_GAMEPAD_USB_PORT);
        //==========================================================

		// Driver Controller -> Command Mapping
		_driverController.leftStick.whileActive(new DriveWithControllers(_driverController.leftStick, _driverController.rightStick));
		_driverController.rightStick.whileActive(new DriveWithControllers(_driverController.leftStick, _driverController.rightStick));
 
		_driverController.leftStick.whenReleased(new DriveWithControllers(_driverController.leftStick, _driverController.rightStick));
		_driverController.rightStick.whenReleased(new DriveWithControllers(_driverController.leftStick, _driverController.rightStick));

        // =========== Operator ======================================
		_operatorController = new BeakXboxController(RobotMap.OPERATOR_GAMEPAD_USB_PORT);
		//==========================================================
		_operatorController.leftStick.whileActive(new LiftClimber(_operatorController.leftStick));
		_operatorController.leftStick.whenReleased(new LiftClimber(_operatorController.leftStick));
		_operatorController.rightStick.whileActive(new DriveClimber(_operatorController.rightStick));
		_operatorController.rightStick.whenReleased(new DriveClimber(_operatorController.rightStick));

		//_operatorController.a.whenPressed(new ToggleBeakPosition());
		_operatorController.a.whenPressed(new PositionClimber());
		_operatorController.b.whenPressed(new TogglePunch());
		_operatorController.y.whenPressed(new AquireHatch());
		_operatorController.x.whenPressed(new ScoreHatch());
		_operatorController.rb.whenPressed(new ToggleStartPos());;
	}
}
