package frc.robot.commands.auton.autons;

import edu.wpi.first.wpilibj.command.CommandGroup;
import frc.robot.auton.pathfollowing.Paths;
import frc.robot.auton.pathfollowing.Paths.Center;
import frc.robot.auton.pathfollowing.control.Path;
import frc.robot.commands.auton.Auton_ParallelStarter;
import frc.robot.commands.auton.Auton_RunMotionProfileAction;
import frc.robot.commands.auton.Auton_RunTimedMotionProfileCommand;
import frc.robot.commands.auton.printTimeFromStart;
import frc.robot.commands.auton.adaptivepaths.CG_FollowVisionPath;
import frc.robot.commands.chassis.DriveSetDistance;
import frc.robot.commands.chassis.InPlaceTurn;

public class TurnTest extends CommandGroup
{
    Path _demoPath = Paths.getPath(Center.DEMO_PATH);
    public TurnTest()
    {
        addParallel(new Auton_ParallelStarter());
        //addSequential(new InPlaceTurn(180, true));
        addSequential(new Auton_RunTimedMotionProfileCommand(_demoPath, 7));
        addSequential(new printTimeFromStart());
        addSequential(new CG_FollowVisionPath());
        // addSequential(new Auton_RunMotionProfileAction(Paths.getPath(Center.R_SWITCH)));
        // addSequential(new Auton_RunMotionProfileAction(Paths.getPath(Center.L_SWITCH)));
        // addSequential(new DriveSetDistance(30));
    }
}