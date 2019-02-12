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
 

public class CG_FollowFirstVisionPath extends CommandGroup {
    VisionLL _limeLight = VisionLL.getInstance();
    GyroNavX _navX = GyroNavX.getInstance();
    double timeOut = 10;
    boolean isFirstCycle = true;

 
    
    public CG_FollowFirstVisionPath(SCORING_TARGET target, SIDE side){
        setInterruptible(false);       
        addSequential(new PrintCommand("VISION PATH PLANNED"));
        addSequential(new printTimeFromStart());
        addSequential(new Auton_turnFromVision());
        addSequential(new PrintCommand("VISION TURN TERMINATING"));
        addSequential(new printTimeFromStart());
        addSequential(new Auton_RunProfileFromVision());
        addSequential(new PrintCommand("VISION PATH TERMINATING"));
        addSequential(new printTimeFromStart());
    }

@Override
    protected void initialize() {
        super.initialize();
        isFirstCycle = true;
    }

    @Override
    protected boolean isFinished() {
        if (isFirstCycle){
            isFirstCycle = false;
            // return (_limeLight.get_distanceToTargetInInches() < 80) || (Math.abs(_limeLight.get_angle1InDegrees()) < 15);
            return false;
        } else {
            return super.isFinished();
        }
    }

}