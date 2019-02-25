package frc.robot.ux;

import frc.robot.RobotMap;
import frc.robot.commands.camera.SwitchCamera;
import frc.robot.commands.chassis.DriveWithControllers;
import frc.robot.commands.elevator.MoveToPresetPosition;
import frc.robot.commands.infeed.AquireHatch;
import frc.robot.commands.infeed.ReleaseInfeed;
import frc.robot.commands.infeed.RunInfeedMotor;
import frc.robot.commands.infeed.ScoreHatch;
import frc.robot.commands.infeed.ToggleBeakInOut;
import frc.robot.commands.infeed.ToggleBeakOpenClose;
import frc.robot.commands.infeed.TogglePunch;
import frc.robot.subsystems.Elevator.ELEVATOR_TARGET_POSITION;
import frc.robot.util.BeakXboxController;

/**
 * This class interfaces with the Driver/Operator Station Lead Student:
 */
public class OI {
	private BeakXboxController _driverController;
	private BeakXboxController _operatorController;
	private BeakXboxController _engineerController;

	// =====================================================================================
	// Define Singleton Pattern
	// =====================================================================================
	private static OI _instance = new OI();

	public static OI getInstance() {
		return _instance;
	}

	// private constructor for singleton pattern
	private OI() {
		// =========== Driver ======================================
		_driverController = new BeakXboxController(RobotMap.DRIVER_GAMEPAD_USB_PORT);
		// ==========================================================

		// Driver Controller -> Command Mapping
		_driverController.leftStick
				.whileActive(new DriveWithControllers(_driverController.leftStick, _driverController.rightStick));
		_driverController.rightStick
				.whileActive(new DriveWithControllers(_driverController.leftStick, _driverController.rightStick));

		_driverController.leftStick
				.whenReleased(new DriveWithControllers(_driverController.leftStick, _driverController.rightStick));
		_driverController.rightStick
				.whenReleased(new DriveWithControllers(_driverController.leftStick, _driverController.rightStick));
		_driverController.a.whenPressed(new MoveToPresetPosition(ELEVATOR_TARGET_POSITION.HOME));
		_driverController.b.whenPressed(new MoveToPresetPosition(ELEVATOR_TARGET_POSITION.HATCH_LEVEL_2));
		// =========== Operator ======================================
		_operatorController = new BeakXboxController(RobotMap.OPERATOR_GAMEPAD_USB_PORT);
		// ==========================================================
		_operatorController.leftStick.whileActive(new RunInfeedMotor(_operatorController.leftStick));
		_operatorController.leftStick.whenReleased(new RunInfeedMotor(_operatorController.leftStick));

		_operatorController.lb.whenPressed(new ToggleBeakOpenClose());
		_operatorController.b.whenPressed(new TogglePunch());
		_operatorController.y.whenPressed(new AquireHatch());
		_operatorController.x.whenPressed(new ScoreHatch());
		_operatorController.rb.whenPressed(new ToggleBeakInOut());
		;
		_operatorController.a.whenPressed(new ReleaseInfeed());
		
		_operatorController.start.whenPressed(new SwitchCamera());

		// =========== Engineer ======================================
		_engineerController = new BeakXboxController(RobotMap.ENGINEERING_GAMEPAD_USB_PORT);
		//============================================================
		_engineerController.dPad.upRight.whenPressed(new MoveToPresetPosition(ELEVATOR_TARGET_POSITION.HATCH_LEVEL_3));
		_engineerController.dPad.right.whenPressed(new MoveToPresetPosition(ELEVATOR_TARGET_POSITION.HATCH_LEVEL_2));
		_engineerController.dPad.downLeft.whenPressed(new MoveToPresetPosition(ELEVATOR_TARGET_POSITION.HATCH_LEVEL_1));
		_engineerController.dPad.down.whenPressed(new MoveToPresetPosition(ELEVATOR_TARGET_POSITION.HOME));
		_engineerController.dPad.downLeft.whenPressed(new MoveToPresetPosition(ELEVATOR_TARGET_POSITION.CARGO_LEVEL_1));
		_engineerController.dPad.left.whenPressed(new MoveToPresetPosition(ELEVATOR_TARGET_POSITION.CARGO_LEVEL_2));
		_engineerController.dPad.upLeft.whenPressed(new MoveToPresetPosition(ELEVATOR_TARGET_POSITION.CARGO_LEVEL_3));
	}
}
