package frc.robot.commands.auton.adaptivePaths;

import edu.wpi.first.wpilibj.command.Command;
import java.lang.Thread;

import frc.robot.auton.pathfollowing.RobotState;
import frc.robot.auton.pathfollowing.control.Path;
import frc.robot.auton.path_planning.problem;
import frc.robot.auton.pathfollowing.motion.RigidTransform;

public class planPath extends Command
{
    Path[] container = new Path[1];
    Thread t;
    

    public planPath(double a1, double a2, double l) {
        RigidTransform curPose = RobotState.getInstance().getLatestFieldToVehicle().getValue();
        t = new Thread("Computation Thread") {
            public void run(){
                // container[0] = problem.solveFromVisionData(a1, a2, l, curPose);
                problem.pathPlanned = true;
            }
         };
    }

    @Override
    protected void initialize() {
        t.start();
        problem.pathPlanned = false;
    }

    @Override
    protected boolean isFinished() {
        return problem.pathPlanned;   
    }

}