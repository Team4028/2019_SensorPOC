package frc.robot.commands.auton.autons;

import edu.wpi.first.wpilibj.command.CommandGroup;
import frc.robot.auton.pathfollowing.Paths;
import frc.robot.auton.pathfollowing.Paths.Left;
import frc.robot.auton.pathfollowing.control.Path;
import frc.robot.commands.auton.RunMotionProfileCommand;
import frc.robot.commands.auton.adaptivepaths.CG_FollowVisionPath;
import frc.robot.commands.chassis.DriveSetDistance;
import frc.robot.commands.chassis.TurnInPlace;
import frc.robot.sensors.GyroNavX.SCORING_TARGET;
import frc.robot.sensors.GyroNavX.SIDE;

public class LDoubleHatchLSideLSide extends CommandGroup
{
    Path _toFeederStation = Paths.getPath(Left.FROM_FIRST_BAY_TO_FEEDER_STATION);
    public LDoubleHatchLSideLSide()
    {
        addSequential(new LSingleHatchLSide());
        addSequential(new DriveSetDistance(-5));
        addSequential(new TurnInPlace(207, true));
        addSequential(new RunMotionProfileCommand(_toFeederStation));
        addSequential(new CG_FollowVisionPath());
    }
}