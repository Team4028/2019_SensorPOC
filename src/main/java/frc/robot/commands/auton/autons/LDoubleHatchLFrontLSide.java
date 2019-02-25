package frc.robot.commands.auton.autons;

import edu.wpi.first.wpilibj.command.CommandGroup;
import frc.robot.auton.pathfollowing.Paths;
import frc.robot.auton.pathfollowing.Paths.Left;
import frc.robot.auton.pathfollowing.control.Path;
import frc.robot.commands.auton.RunMotionProfileCommand;
import frc.robot.commands.auton.adaptivePaths.CG_FollowVisionPath;
import frc.robot.commands.chassis.DriveSetDistance;
import frc.robot.commands.chassis.TurnInPlace;

public class LDoubleHatchLFrontLSide extends CommandGroup {
    Path _toFeederStation = Paths.getPath(Left.FROM_FRONT_CARGO_SHIP_L_TO_FEEDER_STATION);
    public LDoubleHatchLFrontLSide() {
        addSequential(new LSingleHatchLFront());
        addSequential(new DriveSetDistance(-5));
        addSequential(new TurnInPlace(308, false));
        addSequential(new RunMotionProfileCommand(_toFeederStation));
        addSequential(new CG_FollowVisionPath());
    }
}