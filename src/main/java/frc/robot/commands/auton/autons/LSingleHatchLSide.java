package frc.robot.commands.auton.autons;

import edu.wpi.first.wpilibj.command.CommandGroup;
import frc.robot.auton.pathfollowing.Paths;
import frc.robot.auton.pathfollowing.Paths.Left;
import frc.robot.auton.pathfollowing.control.Path;
import frc.robot.commands.auton.RunMotionProfileCommand;
import frc.robot.commands.auton.adaptivepaths.CG_FollowVisionPath;
import frc.robot.sensors.GyroNavX.SCORING_TARGET;
import frc.robot.sensors.GyroNavX.SIDE;

public class LSingleHatchLSide extends CommandGroup
{
    Path _sidePath = Paths.getPath(Left.TO_LEFT_CARGO_SHIP_FIRST);
    public LSingleHatchLSide()
    {
        addSequential(new RunMotionProfileCommand(_sidePath));
        addSequential(new CG_FollowVisionPath());
    }
}