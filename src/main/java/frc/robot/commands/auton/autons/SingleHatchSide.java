package frc.robot.commands.auton.autons;

import edu.wpi.first.wpilibj.command.CommandGroup;
import frc.robot.auton.pathfollowing.Paths;
import frc.robot.auton.pathfollowing.Paths.Left;
import frc.robot.auton.pathfollowing.control.Path;
import frc.robot.commands.auton.Auton_RunMotionProfileAction;
import frc.robot.commands.auton.adaptivepaths.CG_FollowVisionPath;
import frc.robot.sensors.GyroNavX.SCORING_TARGET;
import frc.robot.sensors.GyroNavX.SIDE;

public class SingleHatchSide extends CommandGroup
{
    Path _sidePath = Paths.getPath(Left.TO_LEFT_CARGO_SHIP_FIRST);
    public SingleHatchSide()
    {
        addSequential(new Auton_RunMotionProfileAction(_sidePath));
        addSequential(new CG_FollowVisionPath(SCORING_TARGET.CARGOSHIP_SIDE_ROCKET, SIDE.LEFT));
    }
}