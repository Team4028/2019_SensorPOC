package frc.robot.commands.auton.adaptivePaths;

import edu.wpi.first.wpilibj.command.Command;
import frc.robot.Constants;
import frc.robot.sensors.DistanceRev2mSensor;
import frc.robot.sensors.VisionLL;
import frc.robot.subsystems.Chassis;

public class DriveVisionDistance extends Command {
    double _inches;
    Chassis _chassis = Chassis.getInstance();
    private static final double OFFSET = 28.5;

    public DriveVisionDistance() {
        setInterruptible(true);
        requires(_chassis);
    }

    @Override
    protected void initialize() {
        setInterruptible(false);
        double distance= DistanceRev2mSensor.getInstance().get_distanceToTargetInInches();
        double llDistance = VisionLL.getInstance().get_distanceToTargetInInches();
        System.out.println("Distance Sensor Distance: " + distance);
        System.out.println("LimeLight Distance: " + llDistance);
        if (distance > 0) {
            _chassis.setMotionMagicCmdInches(Math.max((distance-OFFSET),0));
        } else {
            _chassis.setMotionMagicCmdInches(Math.max((llDistance-OFFSET),0));
        }
        System.out.println("Motion Magic Command Set");
    }
    
    @Override
    protected void execute() {
        _chassis.moveToTargetPosDriveSetDistance();        
    }

    @Override
    protected boolean isFinished() {
        if(Math.abs(_chassis.getLeftPos()-_chassis._leftMtrDriveSetDistanceCmd)<Constants.CHASSIS_DRIVE_SET_DISTANCE_DEADBAND
		&& Math.abs(_chassis.getRightPos()-_chassis._rightMtrDriveSetDistanceCmd)<Constants.CHASSIS_DRIVE_SET_DISTANCE_DEADBAND) {
			System.out.println("Chassis is Finished");
			return true;
		} else {
			return false;
		}
    }

}