package frc.robot.commands.auton.adaptivepaths;


import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj.command.Command;
import edu.wpi.first.wpilibj.smartdashboard.SendableBuilder;
import frc.robot.auton.pathfollowing.PathBuilder;
import frc.robot.auton.pathfollowing.RobotState;
import frc.robot.auton.pathfollowing.motion.Translation;
import frc.robot.sensors.DistanceRev2mSensor;
import frc.robot.sensors.VisionLL;
import frc.robot.subsystems.Chassis;

public class DriveStraightWithVision extends Command
{
    Chassis _chassis = Chassis.getInstance();
    VisionLL _limeLight = VisionLL.getInstance();
    RobotState _robotState = RobotState.getInstance();
    DistanceRev2mSensor _distanceSensor = DistanceRev2mSensor.getInstance();
    public DriveStraightWithVision()
    {

    }

    @Override
    protected void initialize() 
    {
        System.out.println("Distance Away:"+_distanceSensor.get_distanceToTargetInInches());
        System.out.println("Angle Offset:" + _limeLight.get_angle1InDegrees());
        _chassis.SetDynamicWantDrivePath(PathBuilder.buildPathFromWaypoints(PathBuilder.getVisionStraightPathWaypoints(_robotState.getLatestFieldToVehicle().getValue().getTranslation(),_limeLight.get_angle1InDegrees() + _chassis.getHeading(), _distanceSensor.get_distanceToTargetInInches())),false);
        _chassis.updateChassis(Timer.getFPGATimestamp());
        int counter=0;
        while(counter<5)
        {
            counter++;
        }
        
    }

    @Override
    protected void execute() 
    {

    }
    @Override
    protected boolean isFinished() 
    {
        return _distanceSensor.get_distanceToTargetInInches()<7+18;
    }

    @Override
    protected void end() {
        _chassis.stop();
    }

}

