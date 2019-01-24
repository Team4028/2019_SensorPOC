package frc.robot.commands.auton.adaptivePaths;

import edu.wpi.first.wpilibj.command.Command;
import frc.robot.auton.path_planning.problem;
import frc.robot.auton.pathfollowing.RobotState;
import frc.robot.auton.pathfollowing.control.Path;
import frc.robot.auton.pathfollowing.motion.RigidTransform;

public class ezMoneyPlanPath extends Command
{
    Path sol;
    double A1;
    double A2;
    double L;
    RigidTransform curPose;
    

    public ezMoneyPlanPath(double a1, double a2, double l) {
        curPose = RobotState.getInstance().getLatestFieldToVehicle().getValue();
        A1 = a1;
        A2 = a2;
        L = l;
    }

    @Override
    protected void initialize() {
        sol = problem.ezMoneySolveFromVisionData(A1, A2, L, curPose);
        problem._path = sol;
    }

    @Override
    protected boolean isFinished() {
        return true; 
    }

}