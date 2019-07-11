package frc.robot.ux;

import javax.lang.model.util.ElementScanner6;

import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.DriverStation.Alliance;
import edu.wpi.first.wpilibj.command.CommandGroup;
import edu.wpi.first.wpilibj.smartdashboard.SendableChooser;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import frc.robot.commands.auton.DriveOffLevel2Forwards;
import frc.robot.commands.auton.DoNothing;
import frc.robot.commands.auton.DriveOffLevel2;
import frc.robot.interfaces.IBeakSquadSubsystem;
import frc.robot.util.LogDataBE;

/**
 * This class defines all of the auton choosers tha will be available on the Dashboard
 */
public class AutonChoosers implements IBeakSquadSubsystem {

    private enum AUTON_MODE {
		UNDEFINED,
		DO_NOTHING, 
		GG_EZ_AUTON_AWARD_SPONSORED_BY_FORD_TWO_HATCH_SANDSTORM_TELEOP
    }

	private enum STARTING_SIDE
	{
		LVL_1,
		LVL_2
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
		_autonAction.addOption("Teleop", AUTON_MODE.GG_EZ_AUTON_AWARD_SPONSORED_BY_FORD_TWO_HATCH_SANDSTORM_TELEOP);
        
        // Auton Starting Side
		_autonStartingSideChooser.setDefaultOption("LEVEL 1", STARTING_SIDE.LVL_1);
		_autonStartingSideChooser.setDefaultOption("LEVEL 2", STARTING_SIDE.LVL_2);


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
				if(startingSide==STARTING_SIDE.LVL_2)
				{
					return new DriveOffLevel2();
				}
				else
				{
					return new DoNothing();
				}
			case GG_EZ_AUTON_AWARD_SPONSORED_BY_FORD_TWO_HATCH_SANDSTORM_TELEOP:
				if(startingSide==STARTING_SIDE.LVL_2)
				{
					return new DriveOffLevel2();
				}
				else
				{
					return new DoNothing();
				}
			
 			default:
				return new DoNothing(); 
		}
	}
	public boolean getIsSafe()
	{
		return _autonStartingSideChooser.getSelected()==STARTING_SIDE.LVL_1;
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
