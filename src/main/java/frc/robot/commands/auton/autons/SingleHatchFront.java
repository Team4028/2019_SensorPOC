package frc.robot.commands.auton.autons;

import edu.wpi.first.wpilibj.command.CommandGroup;
import frc.robot.auton.pathfollowing.Paths;
import frc.robot.auton.pathfollowing.Paths.Left;
import frc.robot.auton.pathfollowing.control.Path;
import frc.robot.commands.auton.Auton_RunMotionProfileAction;
import frc.robot.commands.auton.adaptivepaths.CG_FollowVisionPath;
import frc.robot.sensors.GyroNavX.SCORING_TARGET;
import frc.robot.sensors.GyroNavX.SIDE;

public class SingleHatchFront extends CommandGroup
{
    Path _toCargoShipFront = Paths.getPath(Left.TO_FRONT_CARGO_SHIP_L);
    public SingleHatchFront()
    {
        addSequential(new Auton_RunMotionProfileAction(_toCargoShipFront));
        addSequential(new CG_FollowVisionPath(SCORING_TARGET.CARGOSHIP_FRONT, SIDE.LEFT));
    }
}