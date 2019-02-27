package frc.robot.ux;

import frc.robot.RobotMap;
import frc.robot.commands.camera.SwitchCamera;
import frc.robot.commands.chassis.DriveWithControllers;
import frc.robot.commands.elevator.MoveToPresetPosition;
import frc.robot.commands.infeed.AquireHatch;
import frc.robot.commands.infeed.ReleaseInfeed;
import frc.robot.commands.infeed.RunInfeedMotor;
import frc.robot.commands.infeed.ScoreHatch;
import frc.robot.commands.infeed.ToggleBeakPosition;
import frc.robot.commands.infeed.TogglePunch;
import frc.robot.commands.infeed.ToggleStartPos;
import frc.robot.commands.teleop.TeleopVisionAssistedDriving;
import frc.robot.commands.vision.ChangeVisionPipeline;
import frc.robot.subsystems.Elevator.ELEVATOR_TARGET_POSITION;
import frc.robot.util.BeakXboxController;

/**
 * This class interfaces with the Driver/Operator Station 
 * 	Lead Student:
 */
public class OI {
    private BeakXboxController _driverController;
	private BeakXboxController _operatorController;
	private BeakXboxController _engineerController;
    
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
		_operatorController.leftStick.whileActive(new RunInfeedMotor(_operatorController.leftStick));
		_operatorController.leftStick.whenReleased(new RunInfeedMotor(_operatorController.leftStick));

		_operatorController.lb.whenPressed(new ToggleBeakPosition());
		_operatorController.rb.whenPressed(new MoveToPresetPosition(ELEVATOR_TARGET_POSITION.HOME));
		_operatorController.x.whenPressed(new MoveToPresetPosition(ELEVATOR_TARGET_POSITION.LEVEL_1));
		_operatorController.b.whenPressed(new MoveToPresetPosition(ELEVATOR_TARGET_POSITION.LEVEL_2));
		_operatorController.y.whenPressed(new MoveToPresetPosition(ELEVATOR_TARGET_POSITION.LEVEL_3));
		//_operatorController.a.whenPressed();

		_operatorController.rt.whenActive(new TeleopVisionAssistedDriving(_operatorController.dPad.up, _operatorController.dPad.upRight, _operatorController.dPad.upLeft, _operatorController.rt, _operatorController.a));
		
		_operatorController.start.whenPressed(new SwitchCamera());
		//_operatorController.back.whenPressed(command);

		// =========== Engineer ======================================
		_engineerController = new BeakXboxController(RobotMap.ENGINEERING_GAMEPAD_USB_PORT);
		//============================================================
	}
}
