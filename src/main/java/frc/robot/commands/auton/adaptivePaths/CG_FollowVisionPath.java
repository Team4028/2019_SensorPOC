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
import frc.robot.commands.auton.adaptivepaths.planPath;
import frc.robot.commands.chassis.DriveSetDistance;
import frc.robot.sensors.GyroNavX;
import frc.robot.sensors.VisionLL;
import frc.robot.commands.auton.adaptivepaths.ezMoneyPlanPath;
 

public class CG_FollowVisionPath extends CommandGroup {
    VisionLL _limeLight = VisionLL.getInstance();
    GyroNavX _navX = GyroNavX.getInstance();
    double timeOut = 10;
 
    
    public CG_FollowVisionPath(){
        addParallel(new Auton_ParallelStarter());
        addSequential(new ezMoneyPlanPath(10));
        addSequential(new PrintCommand("Planned First Path"));
        addSequential(new Auton_turnFromVision());
        addSequential(new PrintCommand("Turned First Time"));
        addSequential(new Auton_RunProfileFromVision(5.));
        addSequential(new PrintCommand("Ran First path"));
        addSequential(new planSecondPath());
        addSequential(new PrintCommand("Planned Second Path"));
        addSequential(new Auton_turnFromVision());
        addSequential(new PrintCommand("Turned Second Time"));
        addSequential(new DriveVisionDistance(), 2);
        //addSequential(new Auton_RunProfileFromVision(2.5));

        addSequential(new PrintCommand("Yeeted Entirely"));
        // addSequential(new resetRobotPose());
        //addSequential(new printTimeFromStart());
        //addSequential(new Auton_ParallelStarter());
        //addSequential(new PrintCommand("Planned"));
        // addSequential(new DriveStraightWithVision());
    }

}