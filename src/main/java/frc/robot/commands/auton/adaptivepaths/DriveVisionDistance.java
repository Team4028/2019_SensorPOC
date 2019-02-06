package frc.robot.commands.auton.adaptivepaths;

import edu.wpi.first.wpilibj.command.Command;
import frc.robot.Constants;
import frc.robot.auton.path_planning.problem;
import frc.robot.sensors.DistanceRev2mSensor;
import frc.robot.sensors.VisionLL;
import frc.robot.subsystems.Chassis;

public class DriveVisionDistance extends Command
{
    Chassis _chassis = Chassis.getInstance();
    DistanceRev2mSensor _distanceSensor = DistanceRev2mSensor.getInstance();
    VisionLL _limeLight = VisionLL.getInstance();
    boolean isFirstCycle = true;

    public DriveVisionDistance(){}

    @Override
    protected void initialize() 
    {
        double distance= _distanceSensor.get_distanceToTargetInInches();
        double llDistance = _limeLight.get_distanceToTargetInInches();
        System.out.println("Distance Sensor Distance: " + distance);
        System.out.println("LimeLight Distance: " + llDistance);
        _chassis.setMotionMagicCmdInches(distance-22);

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