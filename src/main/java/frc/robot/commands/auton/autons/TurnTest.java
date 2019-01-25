package frc.robot.commands.auton.autons;

import edu.wpi.first.wpilibj.command.CommandGroup;
import frc.robot.auton.pathfollowing.Paths;
import frc.robot.auton.pathfollowing.Paths.Center;
import frc.robot.commands.auton.Auton_ParallelStarter;
import frc.robot.commands.auton.Auton_RunMotionProfileAction;
import frc.robot.commands.auton.adaptivePaths.CG_FollowVisionPath;
import frc.robot.commands.chassis.DriveSetDistance;
import frc.robot.commands.chassis.InPlaceTurn;

public class TurnTest extends CommandGroup
{
    public TurnTest()
    {
        addParallel(new Auton_ParallelStarter());
        //addSequential(new InPlaceTurn(180, true));
        //addSequential(new DriveSetDistance(50));
        //addSequential(new CG_FollowVisionPath(Math.PI/12, Math.PI/15, 70, true));
        addSequential(new Auton_RunMotionProfileAction(Paths.getPath(Center.L_SWITCH)));
    }
}