package frc.robot.ux;

import frc.robot.RobotMap;
import frc.robot.commands.camera.SwitchCamera;
import frc.robot.commands.chassis.DriveWithControllers;
import frc.robot.commands.climber.DriveClimber;
import frc.robot.commands.climber.LiftClimber;
import frc.robot.commands.elevator.MoveToPresetPosition;
import frc.robot.commands.infeed.AcquireHatch;
import frc.robot.commands.infeed.ReleaseInfeed;
import frc.robot.commands.infeed.RunInfeedMotor;
import frc.robot.commands.infeed.ScoreHatch;
import frc.robot.commands.infeed.ToggleBeakOpen;
import frc.robot.commands.infeed.ToggleBeakPosition;
import frc.robot.commands.infeed.TogglePunch;
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
		_driverController.leftStick.whenReleased(new DriveWithControllers(_driverController.leftStick, _driverController.rightStick));
		_driverController.rightStick.whileActive(new DriveWithControllers(_driverController.leftStick, _driverController.rightStick));	
		_driverController.rightStick.whenReleased(new DriveWithControllers(_driverController.leftStick, _driverController.rightStick));
		_driverController.lb.whenPressed(new AcquireHatch());
		_driverController.lt.whileActive(new RunInfeedMotor(_driverController.lt, false));
		_driverController.lt.whenReleased(new RunInfeedMotor(_driverController.lt, false));
		_driverController.rb.whenPressed(new ScoreHatch());
		_driverController.rt.whileActive(new RunInfeedMotor(_driverController.rt, true));
		_driverController.rt.whenReleased(new RunInfeedMotor(_driverController.rt, true));
		_driverController.a.whenPressed(new ToggleBeakPosition());
		_driverController.b.whenPressed(new ReleaseInfeed());
		_driverController.x.whenPressed(new ToggleBeakOpen());
		_driverController.y.whenPressed(new TogglePunch());
		// =========== Operator ======================================
		_operatorController = new BeakXboxController(RobotMap.OPERATOR_GAMEPAD_USB_PORT);
		//==========================================================
		_operatorController.a.whenPressed(new MoveToPresetPosition(ELEVATOR_TARGET_POSITION.HATCH_LEVEL_1));
		_operatorController.b.whenPressed(new MoveToPresetPosition(ELEVATOR_TARGET_POSITION.CARGO_LEVEL_1));
		_operatorController.x.whenPressed(new MoveToPresetPosition(ELEVATOR_TARGET_POSITION.HATCH_LEVEL_2));
		_operatorController.y.whenPressed(new MoveToPresetPosition(ELEVATOR_TARGET_POSITION.CARGO_LEVEL_2));
		_operatorController.rb.whenPressed(new MoveToPresetPosition(ELEVATOR_TARGET_POSITION.HATCH_LEVEL_3));
		_operatorController.lb.whenPressed(new MoveToPresetPosition(ELEVATOR_TARGET_POSITION.CARGO_LEVEL_3));
		

		// =========== Engineer ======================================
		// _engineerController = new BeakXboxController(RobotMap.ENGINEERING_GAMEPAD_USB_PORT);
		// //============================================================
		// _engineerController.dPad.upRight.whenPressed(new MoveToPresetPosition(ELEVATOR_TARGET_POSITION.HATCH_LEVEL_3));
		// _engineerController.dPad.right.whenPressed(new MoveToPresetPosition(ELEVATOR_TARGET_POSITION.HATCH_LEVEL_2));
		// _engineerController.dPad.downLeft.whenPressed(new MoveToPresetPosition(ELEVATOR_TARGET_POSITION.HATCH_LEVEL_1));
		// _engineerController.dPad.down.whenPressed(new MoveToPresetPosition(ELEVATOR_TARGET_POSITION.HOME));
		// _engineerController.dPad.downLeft.whenPressed(new MoveToPresetPosition(ELEVATOR_TARGET_POSITION.CARGO_LEVEL_1));
		// _engineerController.dPad.left.whenPressed(new MoveToPresetPosition(ELEVATOR_TARGET_POSITION.CARGO_LEVEL_2));
		// _engineerController.dPad.upLeft.whenPressed(new MoveToPresetPosition(ELEVATOR_TARGET_POSITION.CARGO_LEVEL_3));
	}
}

