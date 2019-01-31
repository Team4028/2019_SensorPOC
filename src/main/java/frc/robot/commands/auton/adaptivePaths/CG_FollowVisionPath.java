package frc.robot.commands.auton.adaptivePaths;

import edu.wpi.first.wpilibj.command.CommandGroup;
import edu.wpi.first.wpilibj.command.PrintCommand;
import edu.wpi.first.wpilibj.command.WaitCommand;
import edu.wpi.first.wpilibj.command.CommandGroup;
import frc.robot.auton.path_planning.problem;
import frc.robot.commands.auton.Auton_ParallelStarter;
import frc.robot.commands.auton.Auton_RunProfileFromVision;
import frc.robot.commands.auton.Auton_turnFromVision;
import frc.robot.commands.auton.printTimeFromStart;
import frc.robot.commands.auton.adaptivePaths.planPath;
import frc.robot.sensors.GyroNavX;
import frc.robot.sensors.VisionLL;
import frc.robot.commands.auton.adaptivePaths.ezMoneyPlanPath;
 

public class CG_FollowVisionPath extends CommandGroup {
    VisionLL _limeLight = VisionLL.getInstance();
    GyroNavX _navX = GyroNavX.getInstance();
    double timeOut = 10;
    
    public CG_FollowVisionPath(){
        addParallel(new Auton_ParallelStarter());
        addSequential(new WaitCommand(10));
        // addSequential(new resetRobotPose());
        addSequential(new ezMoneyPlanPath());
        addSequential(new printTimeFromStart());
        addSequential(new Auton_ParallelStarter());
        addSequential(new PrintCommand("Planned"));
        addSequential(new Auton_turnFromVision());
        addSequential(new Auton_RunProfileFromVision(5.));
    }

}