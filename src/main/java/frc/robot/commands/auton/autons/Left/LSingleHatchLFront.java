package frc.robot.commands.auton.autons.Left;

import edu.wpi.first.wpilibj.command.CommandGroup;
import frc.robot.auton.pathfollowing.Paths;
import frc.robot.auton.pathfollowing.Paths.Left;
import frc.robot.auton.pathfollowing.control.Path;
import frc.robot.commands.auton.RunMotionProfileCommand;


public class LSingleHatchLFront extends CommandGroup {
    Path _frontPath = Paths.getPath(Left.TO_FRONT_CARGO_SHIP_L);
    public LSingleHatchLFront() {
        setInterruptible(false);
        addSequential(new RunMotionProfileCommand(_frontPath));
        //addSequential(new CG_FollowVisionPath(SCORING_TARGET.CARGOSHIP_FRONT, SIDE.LEFT));
    }
}