package frc.robot.commands.auton.autons.Center;

import edu.wpi.first.wpilibj.command.CommandGroup;
import frc.robot.auton.pathfollowing.Paths;
import frc.robot.auton.pathfollowing.Paths.Center;
import frc.robot.auton.pathfollowing.control.Path;
import frc.robot.commands.auton.RunMotionProfileCommand;
import frc.robot.commands.auton.adaptivePaths.CG_FollowVisionPath;

public class CSingleHatchRFront extends CommandGroup {
    Path _toCargoShipFront = Paths.getPath(Center.TO_FRONT_CARGO_SHIP_R);
    public CSingleHatchRFront() {
        addSequential(new RunMotionProfileCommand(_toCargoShipFront));
        addSequential(new CG_FollowVisionPath());
    }
}