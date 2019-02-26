package frc.robot.commands.auton.autons.Right;

import edu.wpi.first.wpilibj.command.CommandGroup;
import frc.robot.auton.pathfollowing.Paths;
import frc.robot.auton.pathfollowing.Paths.Right;
import frc.robot.auton.pathfollowing.control.Path;
import frc.robot.commands.auton.RunMotionProfileCommand;
import frc.robot.commands.auton.adaptivePaths.CG_FollowVisionPath;
import frc.robot.commands.chassis.DriveSetDistance;
import frc.robot.commands.chassis.TurnInPlace;
import frc.robot.sensors.GyroNavX.SCORING_TARGET;
import frc.robot.sensors.GyroNavX.SIDE;

public class RDoubleHatchRFrontRSide extends CommandGroup
{
    Path _toFeederStation = Paths.getPath(Right.TO_FEEDER_STATION_FROM_R_FRONT);
    Path _toFirstBay = Paths.getPath(Right.TO_FIRST_BAY_FROM_FEEDER_STATION);
    public RDoubleHatchRFrontRSide()
    {
        setInterruptible(false);
        addSequential(new RSingleHatchRFront());
        addSequential(new DriveSetDistance(-5));
        addSequential(new TurnInPlace(105, true));
        addSequential(new RunMotionProfileCommand(_toFeederStation));
        addSequential(new CG_FollowVisionPath());
        addSequential(new DriveSetDistance(-5));
        addSequential(new TurnInPlace(340, true));
        addSequential(new RunMotionProfileCommand(_toFirstBay));
        addSequential(new CG_FollowVisionPath());

    }
}