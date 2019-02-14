/*----------------------------------------------------------------------------*/
/* Copyright (c) 2018 FIRST. All Rights Reserved.                             */
/* Open Source Software - may be modified and shared by FRC teams. The code   */
/* must be accompanied by the FIRST BSD license file in the root directory of */
/* the project.                                                               */
/*----------------------------------------------------------------------------*/

package frc.robot.ux;

import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.DriverStation.Alliance;
import edu.wpi.first.wpilibj.command.CommandGroup;
import edu.wpi.first.wpilibj.smartdashboard.SendableChooser;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import frc.robot.commands.auton.autons.DoubleHatch;
import frc.robot.commands.auton.autons.SingleHatchFront;
import frc.robot.commands.auton.autons.SingleHatchSide;
import frc.robot.interfaces.IBeakSquadSubsystem;
import frc.robot.util.LogDataBE;

/**
 * This class defines all of the auton choosers tha will be available on the Dashboard
 */
public class AutonChoosers implements IBeakSquadSubsystem {

    private enum AUTON_MODE {
		UNDEFINED,
		FRONT_HATCH,
		SIDE_HATCH,
		DOUBLE_HATCH,
        DO_NOTHING
    }

    private enum STARTING_SIDE {
		LEFT,
		RIGHT
    }
    
    private SendableChooser<AUTON_MODE> _autonAction = new SendableChooser<>();
    private SendableChooser<STARTING_SIDE> _autonStartingSideChooser = new SendableChooser<>();
    private boolean _isStartingLeft = true;
        
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
		_autonAction.addOption("Front Hatch", AUTON_MODE.FRONT_HATCH);
		_autonAction.addOption("Side Hatch", AUTON_MODE.SIDE_HATCH);
		_autonAction.addOption("Double Hatch", AUTON_MODE.DOUBLE_HATCH);
        
        // Auton Starting Side
		_autonStartingSideChooser.setDefaultOption("LEFT", STARTING_SIDE.LEFT);
		_autonStartingSideChooser.addOption("RIGHT", STARTING_SIDE.RIGHT);

    }
    
    public boolean get_isBlueAlliance() {
		return DriverStation.getInstance().getAlliance() == Alliance.Blue;
    }
    
    /** Returns the autonBase object associated with the auton selected on the dashboard */
	public CommandGroup getSelectedAuton() {
		_isStartingLeft = (_autonStartingSideChooser.getSelected() == STARTING_SIDE.LEFT);
		
		switch(_autonAction.getSelected()) {
			case DO_NOTHING:
				return null;
			case FRONT_HATCH:
				return new SingleHatchFront();
			case SIDE_HATCH:
				return new SingleHatchSide();
			case DOUBLE_HATCH:
				return new DoubleHatch();
			default:
				return null; 
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
