package frc.robot.commands.auton.autons.Left;

import edu.wpi.first.wpilibj.command.CommandGroup;
import frc.robot.auton.pathfollowing.Paths;
import frc.robot.auton.pathfollowing.Paths.Left;
import frc.robot.auton.pathfollowing.control.Path;
import frc.robot.commands.auton.RunMotionProfileCommand;
import frc.robot.commands.auton.adaptivePaths.AutoAcquireHatch;
import frc.robot.commands.auton.util.printTimeFromStart;

public class LDoubleHatchRocketL extends CommandGroup
{
    Path _awayFromRocket = Paths.getPath(Left.AWAY_FROM_BACK_ROCKET);
    Path _toFeederStation = Paths.getPath(Left.TO_FEEDER_STATION_FROM_ROCKET);
    public LDoubleHatchRocketL()
    {
        addSequential(new LSingleHatchBackRocketL());
        addSequential(new RunMotionProfileCommand(_awayFromRocket));
        addSequential(new RunMotionProfileCommand(_toFeederStation));
        addSequential(new AutoAcquireHatch());
        addSequential(new printTimeFromStart());



    }
}