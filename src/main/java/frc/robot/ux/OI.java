package frc.robot.ux;

import frc.robot.RobotMap;
import frc.robot.commands.auton.adaptivePaths.AutoAcquireHatch;
import frc.robot.commands.auton.adaptivePaths.AutoPlaceHatch;
import frc.robot.commands.auton.adaptivePaths.AutonFastPlaceHatch;
import frc.robot.commands.camera.SwitchCamera;
import frc.robot.commands.chassis.DriveWithControllers;
import frc.robot.commands.climber.ClimbSequence;
import frc.robot.commands.climber.DriveClimber;
import frc.robot.commands.climber.HoldClimber;
import frc.robot.commands.climber.LiftClimber;
import frc.robot.commands.elevator.MoveToPresetPosition;
import frc.robot.commands.infeed.AcquireHatch;
import frc.robot.commands.infeed.ReleaseInfeed;
import frc.robot.commands.infeed.RetainBall;
import frc.robot.commands.infeed.RunInfeedMotor;
import frc.robot.commands.infeed.ScoreHatch;
import frc.robot.commands.infeed.ToggleBeakInOut;
import frc.robot.commands.infeed.ToggleBeakOpen;
import frc.robot.commands.infeed.TogglePunch;
import frc.robot.commands.teleop.StorePresetElevatorPosition;
import frc.robot.commands.vision.ChangeVisionPipeline;
import frc.robot.subsystems.Elevator;
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

		_driverController.lt.whileActive(new RunInfeedMotor(_driverController.lt, false));
		_driverController.lt.whenReleased(new RetainBall());
		_driverController.rt.whileActive(new RunInfeedMotor(_driverController.rt, true));
		_driverController.rt.whenReleased(new RunInfeedMotor(_driverController.rt, true));
		
		_driverController.lb.whileHeld(new AutoAcquireHatch());
		_driverController.rb.whenPressed(new AutonFastPlaceHatch());

		_driverController.a.whenPressed(new ToggleBeakInOut());
		_driverController.b.whenPressed(new ReleaseInfeed());
		_driverController.x.whenPressed(new ToggleBeakOpen());
		_driverController.y.whenPressed(new TogglePunch());
		
		// =========== Operator ======================================
		_operatorController = new BeakXboxController(RobotMap.OPERATOR_GAMEPAD_USB_PORT);
		// ==========================================================
		_operatorController.leftStick.whileActive(new RunInfeedMotor(_operatorController.leftStick));
		_operatorController.rb.whenPressed(new MoveToPresetPosition(ELEVATOR_TARGET_POSITION.HOME, _operatorController.rt));
		_operatorController.x.whenPressed(new MoveToPresetPosition(ELEVATOR_TARGET_POSITION.LEVEL_1, _operatorController.rt));
		_operatorController.b.whenPressed(new MoveToPresetPosition(ELEVATOR_TARGET_POSITION.LEVEL_2, _operatorController.rt));
		_operatorController.y.whenPressed(new MoveToPresetPosition(ELEVATOR_TARGET_POSITION.LEVEL_3, _operatorController.rt));
		_operatorController.lb.whenPressed(new MoveToPresetPosition(ELEVATOR_TARGET_POSITION.CARGO_ACQUIRE, _operatorController.rt));
		_operatorController.a.whenPressed(new AutoPlaceHatch(_operatorController.rt));

		//_operatorController.rt.whenActive(new TeleopVisionAssistedDriving(_operatorController.dPad.up, _operatorController.dPad.upRight, _operatorController.dPad.upLeft, _operatorController.rt));
		_operatorController.rt.whileActive(new ChangeVisionPipeline(_operatorController.dPad.up, _operatorController.dPad.upRight, _operatorController.dPad.upLeft, _operatorController.rt));
		_operatorController.rt.whenReleased(new ChangeVisionPipeline(_operatorController.dPad.up, _operatorController.dPad.upRight, _operatorController.dPad.upLeft, _operatorController.rt));

		_operatorController.rt.whileActive(new StorePresetElevatorPosition(_operatorController.b, _operatorController.x, _operatorController.y,_operatorController.rb));
		_operatorController.back.whenPressed(new ClimbSequence());
		_operatorController.start.whenPressed(new SwitchCamera());
		//_operatorController.back.whenPressed(command);

		// =========== Engineer ======================================
		_engineerController = new BeakXboxController(RobotMap.ENGINEERING_GAMEPAD_USB_PORT);
		//============================================================
	}
}

