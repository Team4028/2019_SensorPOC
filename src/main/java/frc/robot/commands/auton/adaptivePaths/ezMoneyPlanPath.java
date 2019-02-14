package frc.robot.commands.auton.adaptivepaths;

import edu.wpi.first.wpilibj.command.Command;
import edu.wpi.first.wpilibj.interfaces.Gyro;
import frc.robot.auton.path_planning.problem;
import frc.robot.auton.pathfollowing.RobotState;
import frc.robot.auton.pathfollowing.control.Path;
import frc.robot.auton.pathfollowing.motion.RigidTransform;
import frc.robot.auton.pathfollowing.motion.Rotation;
import frc.robot.auton.pathfollowing.motion.Translation;
import frc.robot.sensors.GyroNavX;
import frc.robot.sensors.VisionLL;
import frc.robot.sensors.GyroNavX.SCORING_TARGET;
import frc.robot.sensors.GyroNavX.SIDE;

public class ezMoneyPlanPath extends Command
{
    Path sol;
    Path iPath;
    double A1;
    double A2;
    double L;
    RigidTransform curPose;
    VisionLL _limeLight = VisionLL.getInstance();
    GyroNavX _navX = GyroNavX.getInstance();
    int _numTries;
    int tries = 0;
    boolean hasSeenTarget = false;
    SCORING_TARGET _target;
    SIDE _side;

    public ezMoneyPlanPath(int numTries, SCORING_TARGET target, SIDE side) 
    {
        iPath =  null; //problem._path;
        _numTries = numTries;
        _target = target;
        _side = side;
        setInterruptible(false);


    }

    @Override
    protected void initialize() {}

    @Override
    protected void execute()
    {
        if (!(hasSeenTarget))
        {
            if (_limeLight.get_isTargetInFOV())
            {
                hasSeenTarget = true;
                double A1 = _limeLight.get_angle1InDegrees();
                double A2= _navX.get_angle2InDegreesFromLL(_target, _side);
                double distance= _limeLight.get_distanceToTargetInInches();
                RigidTransform rt = RobotState.getInstance().getLatestFieldToVehicle().getValue();
                curPose = new RigidTransform(new Translation(rt.getTranslation().x(), rt.getTranslation().y()), Rotation.fromDegrees(_navX.getYaw()));
                System.out.println("ANGLE 1: " + A1);
                System.out.println("ANGLE 2: " + A2);
                System.out.println("DISTANCE: " + distance);
                problem.planPathFromVisionData(A1, A2, distance, curPose);
                System.out.println("PLAN PATH FROM VISION DATA COMPLETE");
            } 
            else 
            {
                tries++;
                System.out.println("Tries: " + tries);
            }
        } 
    }

    @Override
    protected boolean isFinished() {
        return (!(problem._path == iPath));
    }

    @Override
    protected void  end(){
        System.out.println("Path Plan Command End Running");
    }

}