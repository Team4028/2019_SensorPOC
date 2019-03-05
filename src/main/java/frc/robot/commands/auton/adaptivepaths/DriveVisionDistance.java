package frc.robot.commands.auton.adaptivePaths;

import edu.wpi.first.wpilibj.command.Command;
import frc.robot.Constants;
import frc.robot.sensors.DistanceRev2mSensor;
import frc.robot.sensors.VisionLL;
import frc.robot.sensors.GyroNavX.SCORING_TARGET;
import frc.robot.sensors.GyroNavX.SIDE;
import frc.robot.subsystems.Chassis;

public class DriveVisionDistance extends Command {
    double _inches;
    Chassis _chassis = Chassis.getInstance();
    SCORING_TARGET target;
    SIDE side;
    VisionLL _limelight = VisionLL.getInstance();

    private static final double OFFSET = 12;
    boolean _canSeeTarget;
    

    public DriveVisionDistance() {
        setInterruptible(false);
        requires(_chassis);
    }

    @Override
    protected void initialize() {
        if(_limelight.get_isTargetInFOV())
        {
                double dsDistance= DistanceRev2mSensor.getInstance().get_distanceToTargetInInches();
            double llDistance = _limelight.get_revisedDistance();
            System.out.println("Distance Sensor Distance: " + dsDistance);
            System.out.println("LimeLight Distance: " + llDistance);
            if (dsDistance > 0) {
                _chassis.setMotionMagicCmdInches(Math.max((dsDistance-OFFSET),0));
                System.out.println("Distance Sensor Distance Used");
            } else {
                _chassis.setMotionMagicCmdInches(Math.max((llDistance-OFFSET-4),0));
                System.out.println("Limelight Distance Used");
            }
            System.out.println("Motion Magic Command Set");
            _canSeeTarget=true;
        }
        else
        {
            _canSeeTarget = false;
        }
    }
    
    @Override
    protected void execute() {
        _chassis.moveToTargetPosDriveSetDistance();        
    }

    @Override
    protected boolean isFinished() {

        if(_canSeeTarget)
        {
            if(Math.abs(_chassis.getLeftPos()-_chassis._leftMtrDriveSetDistanceCmd)<Constants.CHASSIS_DRIVE_SET_DISTANCE_DEADBAND
            && Math.abs(_chassis.getRightPos()-_chassis._rightMtrDriveSetDistanceCmd)<Constants.CHASSIS_DRIVE_SET_DISTANCE_DEADBAND) {
                System.out.println("Chassis is Finished");
                return true;
            } else {
                return false;
            }
        }
        else
        {
            return true;
        }

    }
    @Override
    protected void end() {
        _chassis.stop();
    }

}