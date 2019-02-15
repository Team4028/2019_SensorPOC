package frc.robot.ux;

import javax.lang.model.util.ElementScanner6;

import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.DriverStation.Alliance;
import edu.wpi.first.wpilibj.command.CommandGroup;
import edu.wpi.first.wpilibj.smartdashboard.SendableChooser;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import frc.robot.commands.auton.autons.DoNothing;
import frc.robot.commands.auton.autons.LineCross;
import frc.robot.commands.auton.autons.Center.CDoubleHatchLFrontLSide;
import frc.robot.commands.auton.autons.Center.CSingleHatchLFront;
import frc.robot.commands.auton.autons.Left.LDoubleHatchLFrontLSide;
import frc.robot.commands.auton.autons.Left.LDoubleHatchLSideLSide;
import frc.robot.commands.auton.autons.Left.LSingleHatchLFront;
import frc.robot.commands.auton.autons.Left.LSingleHatchLSide;
import frc.robot.commands.auton.autons.Right.RSingleHatchRFront;
import frc.robot.commands.auton.autons.Right.RSingleHatchRSide;
import frc.robot.interfaces.IBeakSquadSubsystem;
import frc.robot.util.LogDataBE;

/**
 * This class defines all of the auton choosers tha will be available on the Dashboard
 */
public class AutonChoosers implements IBeakSquadSubsystem {

    private enum AUTON_MODE {
		UNDEFINED,
		LINE_CROSS,
		LEFT_FRONT_HATCH,
		RIGHT_FRONT_HATCH,
		SIDE_HATCH,
		DOUBLE_HATCH_FRONT_SIDE,
		DOUBLE_HATCH_SIDE_SIDE,
        DO_NOTHING, 
    }

	private enum STARTING_SIDE 
	{
		LEFT,
		CENTER,
		RIGHT
    }
    
    private SendableChooser<AUTON_MODE> _autonAction = new SendableChooser<>();
    private SendableChooser<STARTING_SIDE> _autonStartingSideChooser = new SendableChooser<>();
    //private boolean _isStartingLeft = true;
        
    //=====================================================================================
	// Define Singleton Pattern
	//=====================================================================================
	private static AutonChoosers _instance = new AutonChoosers();
	
	public static AutonChoosers getInstance() {
		return _instance;
	}
	
	// private constructor for singleton pattern
	private AutonChoosers() {
        // Auton Mode
		_autonAction.setDefaultOption("Do Nothing", AUTON_MODE.DO_NOTHING);
		_autonAction.addOption("Left Front Hatch", AUTON_MODE.LEFT_FRONT_HATCH);
		_autonAction.addOption("Right Front Hatch", AUTON_MODE.RIGHT_FRONT_HATCH);
		_autonAction.addOption("Side Hatch", AUTON_MODE.SIDE_HATCH);
		_autonAction.addOption("Line Cross", AUTON_MODE.LINE_CROSS);
		_autonAction.addOption("Double Hatch Front Side", AUTON_MODE.DOUBLE_HATCH_FRONT_SIDE);
		_autonAction.addOption("Double Hatch Side Side", AUTON_MODE.DOUBLE_HATCH_SIDE_SIDE);
        
        // Auton Starting Side
		_autonStartingSideChooser.setDefaultOption("LEFT", STARTING_SIDE.LEFT);
		_autonStartingSideChooser.addOption("RIGHT", STARTING_SIDE.RIGHT);
		_autonStartingSideChooser.addOption("CENTER", STARTING_SIDE.CENTER);

    }
    
    public boolean get_isBlueAlliance() {
		return DriverStation.getInstance().getAlliance() == Alliance.Blue;
    }
    
    /** Returns the autonBase object associated with the auton selected on the dashboard */
	public CommandGroup getSelectedAuton() {
		STARTING_SIDE startingSide = _autonStartingSideChooser.getSelected();
		
		switch(_autonAction.getSelected()) {
			case DO_NOTHING:
				return new DoNothing();
			case LINE_CROSS:
				return new LineCross();
			case LEFT_FRONT_HATCH:
				if(startingSide==STARTING_SIDE.LEFT)
				{
					return new LSingleHatchLFront();
				}
				else if(startingSide==STARTING_SIDE.CENTER)
				{
					return new CSingleHatchLFront();
				}
				else if(startingSide==STARTING_SIDE.RIGHT)
				{
					return null;
				}
			case RIGHT_FRONT_HATCH:
				return new RSingleHatchRFront();
			case SIDE_HATCH:
				if(startingSide==STARTING_SIDE.LEFT)
				{
					return new LSingleHatchLSide();
				}
				else if(startingSide==STARTING_SIDE.CENTER)
				{
					return new RSingleHatchRSide();
				}
				else if(startingSide==STARTING_SIDE.RIGHT)
				{
					return new RSingleHatchRSide();
				}
			case DOUBLE_HATCH_SIDE_SIDE:
				return new LDoubleHatchLSideLSide();
			case DOUBLE_HATCH_FRONT_SIDE:
				if (startingSide==STARTING_SIDE.LEFT)
				{
					return new LDoubleHatchLFrontLSide();
				}
				else if(startingSide==STARTING_SIDE.CENTER)
				{
					return new CDoubleHatchLFrontLSide();
				}
				else
				{
					return null;
				}
			default:
				return new DoNothing(); 
		}
	}
	
    @Override
    public void updateLogData(LogDataBE logData) {

    }

    @Override
    public void updateDashboard() {
		SmartDashboard.putData("Auton",_autonAction);
		SmartDashboard.putData("Side Start",_autonStartingSideChooser);
        SmartDashboard.putString("AutonChoosers:AutonAction", _autonAction.getSelected().toString());
        SmartDashboard.putString("AutonChoosers:StartingSide", _autonStartingSideChooser.getSelected().toString());

	}
}
