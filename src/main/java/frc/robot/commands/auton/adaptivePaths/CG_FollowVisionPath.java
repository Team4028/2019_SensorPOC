package frc.robot.commands.auton.adaptivePaths;

import edu.wpi.first.wpilibj.command.CommandGroup;
import edu.wpi.first.wpilibj.command.PrintCommand;
import edu.wpi.first.wpilibj.command.CommandGroup;
import frc.robot.auton.path_planning.problem;
import frc.robot.commands.auton.Auton_ParallelStarter;
import frc.robot.commands.auton.adaptivePaths.planPath;
import frc.robot.commands.auton.adaptivePaths.ezMoneyPlanPath;

public class CG_FollowVisionPath extends CommandGroup {

    double timeOut = 10;
    
    public CG_FollowVisionPath(double a1, double a2, double l, boolean izCool){
        addParallel(new Auton_ParallelStarter());
        if (izCool){
            addSequential(new ezMoneyPlanPath(a1, a2, l));
        } else {
            addSequential(new planPath(a1, a2, l));
        }
        addSequential(new PrintCommand("Planned"));
        //addSequential(new followVisionProfile(timeOut));
    }
}