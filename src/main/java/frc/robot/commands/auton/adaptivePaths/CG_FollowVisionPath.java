package frc.robot.commands.auton.adaptivepaths;

import edu.wpi.first.wpilibj.command.CommandGroup;
import edu.wpi.first.wpilibj.command.PrintCommand;
import edu.wpi.first.wpilibj.command.WaitCommand;
import edu.wpi.first.wpilibj.command.CommandGroup;
import frc.robot.auton.path_planning.problem;
import frc.robot.auton.pathfollowing.Paths;
import frc.robot.auton.pathfollowing.Paths.Center;
import frc.robot.auton.pathfollowing.control.Path;
import frc.robot.commands.auton.Auton_ParallelStarter;
import frc.robot.commands.auton.Auton_RunProfileFromVision;
import frc.robot.commands.auton.Auton_turnFromVision;
import frc.robot.commands.auton.printTimeFromStart;
import frc.robot.commands.chassis.DriveSetDistance;
import frc.robot.sensors.GyroNavX;
import frc.robot.sensors.VisionLL;
import frc.robot.sensors.GyroNavX.SCORING_TARGET;
import frc.robot.sensors.GyroNavX.SIDE;
import frc.robot.commands.auton.adaptivepaths.ezMoneyPlanPath;
 

public class CG_FollowVisionPath extends CommandGroup {
    VisionLL _limeLight = VisionLL.getInstance();
    GyroNavX _navX = GyroNavX.getInstance();
    double timeOut = 10;
 
    
    public CG_FollowVisionPath(SCORING_TARGET target, SIDE side){
        addParallel(new Auton_ParallelStarter());
        addSequential(new PrintCommand("FIRST PATH TERMINATING"));
        addSequential(new FindTarget(target, side));
        addSequential(new PrintCommand("TARGET FOUND"));
        addSequential(new ezMoneyPlanPath(10, target, side));
        addSequential(new PrintCommand("VISION PATH PLANNED"));
        // addSequential(new PrintCommand("PATH PLANNED: " + problem._path.toString()));
        addSequential(new printTimeFromStart());
        addSequential(new Auton_turnFromVision());
        addSequential(new PrintCommand("VISION TURN TERMINATING"));
        addSequential(new printTimeFromStart());
        addSequential(new Auton_RunProfileFromVision());
        addSequential(new PrintCommand("VISION PATH TERMINATING"));
        addSequential(new printTimeFromStart());
        addSequential(new FindTarget(target, side));
        addSequential(new PrintCommand("VISION TARGET FOUND"));
        addSequential(new printTimeFromStart());
        addSequential(new planSecondPath(target, side));
        addSequential(new PrintCommand("SECOND PATH PLANNED"));
        addSequential(new printTimeFromStart());
        addSequential(new Auton_turnFromVision());
        addSequential(new PrintCommand("SECOND VISION TURN TERMINATING"));
        addSequential(new printTimeFromStart());
        addSequential(new DriveVisionDistance(), 1.5);
        addSequential(new PrintCommand("VISION DRIVE STRAIT TERMINATING"));
        addSequential(new printTimeFromStart());

    }

}