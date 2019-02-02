package frc.robot.commands.auton.adaptivepaths;

import edu.wpi.first.wpilibj.command.Command;
import frc.robot.Constants;
import frc.robot.auton.path_planning.problem;
import frc.robot.subsystems.Chassis;

public class DriveVisionDistance extends Command
{
    double _inches;
    Chassis _chassis = Chassis.getInstance();
    public DriveVisionDistance(){}

    @Override
    protected void initialize() 
    {
        _inches = problem._distance;
        _chassis.setMotionMagicCmdInches(_inches-12);
    }
    @Override
    protected void execute() {
        _chassis.moveToTargetPosDriveSetDistance();        
    }
    @Override
    protected boolean isFinished() 
    {
        if(Math.abs(_chassis.getLeftPos()-_chassis._leftMtrDriveSetDistanceCmd)<Constants.CHASSIS_DRIVE_SET_DISTANCE_DEADBAND
		&& Math.abs(_chassis.getRightPos()-_chassis._rightMtrDriveSetDistanceCmd)<Constants.CHASSIS_DRIVE_SET_DISTANCE_DEADBAND)
		{
			System.out.println("Chassis is Finished");
			return true;
		}
		else
		{
			return false;
		}
    }

    @Override
    protected void end() {
        _chassis.stop();
    }

}