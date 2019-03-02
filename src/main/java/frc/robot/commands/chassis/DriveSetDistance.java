package frc.robot.commands.chassis;

import edu.wpi.first.wpilibj.command.Command;
import frc.robot.Constants;
import frc.robot.subsystems.Chassis;

public class DriveSetDistance extends Command {
    double _inches;
    Chassis _chassis = Chassis.getInstance();

    public DriveSetDistance(double inches) {
        _inches=inches;
        requires(_chassis);
        setInterruptible(false);
    }

    @Override
    protected void initialize() {
        System.out.println("Motion Magic Straight Initialized");
        _chassis.setMotionMagicCmdInches(_inches);
    }

    @Override
    protected void execute() {
        _chassis.moveToTargetPosDriveSetDistance();        
    }

    @Override
    protected boolean isFinished() {
        if(Math.abs(_chassis.getLeftPos()-_chassis._leftMtrDriveSetDistanceCmd)<Constants.CHASSIS_DRIVE_SET_DISTANCE_DEADBAND
		&& Math.abs(_chassis.getRightPos()-_chassis._rightMtrDriveSetDistanceCmd)<Constants.CHASSIS_DRIVE_SET_DISTANCE_DEADBAND) {
            return true;
		} else {
            return false;
		}
    }

    @Override
    protected void end() {
        _chassis.stop();
    }
}