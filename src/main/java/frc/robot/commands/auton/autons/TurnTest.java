package frc.robot.commands.auton.autons;

import edu.wpi.first.wpilibj.command.CommandGroup;
import frc.robot.commands.auton.Auton_ParallelStarter;
import frc.robot.commands.auton.adaptivePaths.CG_FollowVisionPath;
import frc.robot.commands.chassis.InPlaceTurn;

public class TurnTest extends CommandGroup
{
    public TurnTest()
    {
        //addSequential(new InPlaceTurn(100, true));
        addParallel(new Auton_ParallelStarter());
        addSequential(new CG_FollowVisionPath(Math.PI/12, Math.PI/15, 70, true));
    }
}