package frc.robot.ux;

import javax.lang.model.util.ElementScanner6;

import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.DriverStation.Alliance;
import edu.wpi.first.wpilibj.command.CommandGroup;
import edu.wpi.first.wpilibj.smartdashboard.SendableChooser;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import frc.robot.commands.auton.DriveOffLevel2Forwards;
import frc.robot.commands.auton.autons.DoNothing;
import frc.robot.commands.auton.autons.Level2TeleopSandstorm;
import frc.robot.commands.auton.autons.LineCross;
import frc.robot.commands.auton.autons.Left.LSingleHatchBackRocketL;
import frc.robot.commands.auton.autons.Left.LSingleHatchLSide;
import frc.robot.commands.auton.autons.Left.LSingleHatchLSideLevel2;
import frc.robot.commands.auton.autons.Right.RSingleHatchBackRocket;
import frc.robot.commands.auton.autons.Right.RSingleHatchRSide;
import frc.robot.commands.auton.autons.Right.RSingleHatchRSideLevel2;
import frc.robot.interfaces.IBeakSquadSubsystem;
import frc.robot.util.LogDataBE;

/**
 * This class defines all of the auton choosers tha will be available on the Dashboard
 */
public class AutonChoosers implements IBeakSquadSubsystem {

    private enum AUTON_MODE {
		UNDEFINED,
		LINE_CROSS,
		SIDE_HATCH,
		ROCKET,
        DO_NOTHING, 
    }

	private enum STARTING_SIDE 
	{
		LEFT_LVL1,
		LEFT_LVL2,
		RIGHT_LVL1,
		RIGHT_LVL2
    }
    private SendableChooser<AUTON_MODE> _autonAction = new SendableChooser<>();
    private SendableChooser<STARTING_SIDE> _autonStartingSideChooser = new SendableChooser<>();
        
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
		_autonAction.addOption("Side Hatch", AUTON_MODE.SIDE_HATCH);
		_autonAction.addOption("Line Cross", AUTON_MODE.LINE_CROSS);
		_autonAction.addOption("Rocket", AUTON_MODE.ROCKET);
        
        // Auton Starting Side
		_autonStartingSideChooser.setDefaultOption("LEFT LEVEL 1", STARTING_SIDE.LEFT_LVL1);
		_autonStartingSideChooser.setDefaultOption("LEFT LEVEL 2", STARTING_SIDE.LEFT_LVL2);
		_autonStartingSideChooser.addOption("RIGHT LEVEL 1", STARTING_SIDE.RIGHT_LVL1);
		_autonStartingSideChooser.addOption("RIGHT LEVEL 2", STARTING_SIDE.RIGHT_LVL2);

    }
    
    public boolean get_isBlueAlliance() {
		return DriverStation.getInstance().getAlliance() == Alliance.Blue;
    }
    
    /** Returns the autonBase object associated with the auton selected on the dashboard */
	public CommandGroup getSelectedAuton() {
		STARTING_SIDE startingSide = _autonStartingSideChooser.getSelected();
		System.out.println("Selecting an Auton");
		switch(_autonAction.getSelected()) {
			case DO_NOTHING:
				if(startingSide==STARTING_SIDE.LEFT_LVL2||startingSide==STARTING_SIDE.LEFT_LVL2)
				{
					return new Level2TeleopSandstorm();
				}
				else
				{
					return new DoNothing();
				}
			case LINE_CROSS:
				return new LineCross();
			
			case SIDE_HATCH:
				if(startingSide==STARTING_SIDE.LEFT_LVL1)
				{
					return new LSingleHatchLSide();
				}
				else if(startingSide==STARTING_SIDE.LEFT_LVL2)
				{
					return new LSingleHatchLSideLevel2();
				}
				else if(startingSide==STARTING_SIDE.RIGHT_LVL1)
				{
					return new RSingleHatchRSide();
				}
				else if (startingSide==STARTING_SIDE.RIGHT_LVL2)
				{
					return new RSingleHatchRSideLevel2();
				}
			case ROCKET:
				if (startingSide == STARTING_SIDE.LEFT_LVL1){
					return new LSingleHatchBackRocketL();
				} else {
					return new RSingleHatchBackRocket();
				}
 			default:
				return new DoNothing(); 
		}
	}
	public boolean getIsSafe()
	{
		return _autonStartingSideChooser.getSelected()==STARTING_SIDE.LEFT_LVL1 || _autonStartingSideChooser.getSelected()==STARTING_SIDE.RIGHT_LVL1;
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
