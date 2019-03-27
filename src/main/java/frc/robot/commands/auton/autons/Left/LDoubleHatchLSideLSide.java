package frc.robot.commands.auton.autons.Left;

import edu.wpi.first.wpilibj.command.CommandGroup;
import frc.robot.auton.pathfollowing.Paths;
import frc.robot.auton.pathfollowing.Paths.Left;
import frc.robot.auton.pathfollowing.control.Path;
import frc.robot.commands.auton.RunMotionProfileCommand;
import frc.robot.commands.auton.adaptivePaths.AutoAcquireHatch;
import frc.robot.commands.auton.adaptivePaths.AutoPlaceHatch;
import frc.robot.commands.auton.adaptivePaths.AutoTrackTarget;
import frc.robot.commands.auton.util.printTimeFromStart;
import frc.robot.commands.chassis.DriveSetDistance;
import frc.robot.commands.chassis.TurnInPlace;
import frc.robot.commands.infeed.AcquireHatch;
import frc.robot.commands.infeed.TogglePunch;
import frc.robot.sensors.GyroNavX.SCORING_TARGET;
import frc.robot.sensors.GyroNavX.SIDE;

public class LDoubleHatchLSideLSide extends CommandGroup {
    Path _toFeederStation = Paths.getPath(Left.FROM_FIRST_BAY_TO_FEEDER_STATION);
    Path _toBay = Paths.getPath(Left.FROM_FEEDER_STATION_TO_SECOND_BAY);
    Path _awayFromFeeder = Paths.getPath(Left.AWAY_FROM_FEEDER);
    public LDoubleHatchLSideLSide() {
        setInterruptible(false);
        addSequential(new LSingleHatchLSide());
        addSequential(new TurnInPlace(-167, true));
        addSequential(new RunMotionProfileCommand(_toFeederStation));
        addSequential(new AutoTrackTarget());
        addSequential(new AcquireHatch());
        addSequential(new printTimeFromStart());
        //addSequential(new TurnInPlace(20, false));
        //quential(new AutoPlaceHatch());
    }
}