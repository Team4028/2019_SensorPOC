package frc.robot.commands.auton.autons.Left;

import edu.wpi.first.wpilibj.command.CommandGroup;
import frc.robot.auton.pathfollowing.Paths;
import frc.robot.auton.pathfollowing.Paths.Left;
import frc.robot.auton.pathfollowing.control.Path;
import frc.robot.commands.auton.RunMotionProfileCommand;
import frc.robot.commands.auton.StartAcquireHatch;
import frc.robot.commands.chassis.TurnInPlace;
import frc.robot.commands.infeed.SendBucketOut;

;

public class LSingleHatchLSide extends CommandGroup {
    Path _sidePath = Paths.getPath(Left.TO_LEFT_CARGO_SHIP_FIRST);
    public LSingleHatchLSide() {
        setInterruptible(false);
        addSequential(new RunMotionProfileCommand(_sidePath));
        addSequential(new TurnInPlace(90, true));
        addSequential(new SendBucketOut());
    }
}