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
    

    public ezMoneyPlanPath() {
        iPath =  null; //problem._path;


    }

    @Override
    protected void initialize() {
        double A1 = _limeLight.get_angle1InDegrees();
        double A2= _navX.get_angle2InDegreesFromLL(SCORING_TARGET.CARGOSHIP_SIDE_ROCKET, SIDE.RIGHT);
        double distance= _limeLight.get_distanceToTargetInInches();
        RigidTransform rt = RobotState.getInstance().getLatestFieldToVehicle().getValue();
        curPose = new RigidTransform(new Translation(rt.getTranslation().x(), rt.getTranslation().y()), Rotation.fromDegrees(_navX.getYaw()));
        problem.planPathFromVisionData(A1, A2, distance, curPose);
    }

    @Override
    protected boolean isFinished() {
        return !(problem._path == iPath);
    }

}