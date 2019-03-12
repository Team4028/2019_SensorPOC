package frc.robot.commands.auton.autons.Center;
import edu.wpi.first.wpilibj.command.CommandGroup;
import frc.robot.auton.pathfollowing.Paths;
import frc.robot.auton.pathfollowing.Paths.Left;
import frc.robot.auton.pathfollowing.control.Path;
import frc.robot.commands.auton.RunMotionProfileCommand;
import frc.robot.commands.auton.adaptivePaths.AutoAcquireHatch;
import frc.robot.commands.auton.adaptivePaths.AutoPlaceHatch;
import frc.robot.commands.auton.util.printTimeFromStart;
import frc.robot.commands.chassis.DriveSetDistance;
import frc.robot.commands.chassis.TurnInPlace;
import frc.robot.sensors.GyroNavX.SCORING_TARGET;
import frc.robot.sensors.GyroNavX.SIDE;

public class CDoubleHatchLFrontLSide extends CommandGroup 
{
    Path _toFeederStation = Paths.getPath(Left.FROM_FRONT_CARGO_SHIP_L_TO_FEEDER_STATION);
    Path _toBay = (Paths.getPath(Left.FROM_FEEDER_STATION_TO_FIRST_BAY));
    Path _awayFromFeeder = Paths.getPath(Left.AWAY_FROM_FEEDER);
    public CDoubleHatchLFrontLSide() 
    {
        setInterruptible(false);
        addSequential(new CSingleHatchLFront());
        addSequential(new TurnInPlace(-90, false));
        addSequential(new RunMotionProfileCommand(_toFeederStation));
        addSequential(new printTimeFromStart());
        addSequential(new AutoAcquireHatch());
        addSequential(new RunMotionProfileCommand(_awayFromFeeder));
        /*addSequential(new DriveSetDistance(-5));
        addSequential(new TurnInPlace(20, false));
        addSequential(new RunMotionProfileCommand(_toBay));
        addSequential(new CG_FollowVisionPath());*/
    }
}