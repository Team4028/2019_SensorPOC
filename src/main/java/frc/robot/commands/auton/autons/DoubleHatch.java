package frc.robot.commands.auton.autons;

import edu.wpi.first.wpilibj.command.CommandGroup;
import frc.robot.auton.pathfollowing.Paths;
import frc.robot.auton.pathfollowing.Paths.Left;
import frc.robot.auton.pathfollowing.control.Path;
import frc.robot.commands.auton.Auton_RunMotionProfileAction;
import frc.robot.commands.auton.adaptivepaths.CG_FollowVisionPath;
import frc.robot.commands.chassis.DriveSetDistance;
import frc.robot.commands.chassis.InPlaceTurn;
import frc.robot.sensors.GyroNavX.SCORING_TARGET;
import frc.robot.sensors.GyroNavX.SIDE;

public class DoubleHatch extends CommandGroup
{
    Path _toFeederStation = Paths.getPath(Left.FROM_FIRST_BAY_TO_FEEDER_STATION);
    public DoubleHatch()
    {
        addSequential(new SingleHatchSide());
        addSequential(new DriveSetDistance(-5));
        addSequential(new InPlaceTurn(270, true));
        addSequential(new Auton_RunMotionProfileAction(_toFeederStation));
        addSequential(new CG_FollowVisionPath(SCORING_TARGET.FEEDER_STATION, SIDE.LEFT));
    }
}