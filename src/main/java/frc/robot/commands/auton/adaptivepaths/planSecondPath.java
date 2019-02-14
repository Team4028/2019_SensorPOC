package frc.robot.commands.auton.adaptivepaths;

import edu.wpi.first.wpilibj.command.Command;
import frc.robot.auton.path_planning.problem;
import frc.robot.auton.pathfollowing.RobotState;
import frc.robot.auton.pathfollowing.control.Path;
import frc.robot.auton.pathfollowing.motion.RigidTransform;
import frc.robot.auton.pathfollowing.motion.Rotation;
import frc.robot.auton.pathfollowing.motion.Translation;
import frc.robot.sensors.DistanceRev2mSensor;
import frc.robot.sensors.GyroNavX;
import frc.robot.sensors.VisionLL;
import frc.robot.sensors.GyroNavX.SCORING_TARGET;
import frc.robot.sensors.GyroNavX.SIDE;

public class planSecondPath extends Command
{
    Path sol;
    Path iPath;
    double A1;
    double A2;
    double L;
    RigidTransform curPose;
    VisionLL _limeLight = VisionLL.getInstance();
    GyroNavX _navX = GyroNavX.getInstance();
    DistanceRev2mSensor _distanceSensor = DistanceRev2mSensor.getInstance();
    SCORING_TARGET _target;
    SIDE _side;
    double _initialTheta;
    

    public planSecondPath(SCORING_TARGET target, SIDE side) {
        setInterruptible(false);
        _target = target;
        _side = side;
        _initialTheta = problem._theta;
        //iPath =  problem._path;
    }

    @Override
    protected void initialize() 
    {
        double A1 = _limeLight.get_angle1InDegrees();
        double A2= _navX.get_angle2InDegreesFromLL(_target, _side);
        double distance= _distanceSensor.get_distanceToTargetInInches();
        if(distance<0)
        {
            distance = _limeLight.get_distanceToTargetInInches();
            
        }        
        System.out.println("Angle 1: " + A1);
        System.out.println("Heading: " + _navX.getYaw());
        RigidTransform rt = RobotState.getInstance().getLatestFieldToVehicle().getValue();
        curPose = new RigidTransform(new Translation(rt.getTranslation().x(), rt.getTranslation().y()), Rotation.fromDegrees(_navX.getYaw()));
        problem.planSecondPathFromVisionData(A1, A2, distance, curPose);
    }

    @Override
    protected boolean isFinished() {
        return !(problem._theta == _initialTheta);
    }

}