package frc.robot.commands.auton.autons.Right;

import edu.wpi.first.wpilibj.command.CommandGroup;
import frc.robot.auton.pathfollowing.Paths;
import frc.robot.auton.pathfollowing.Paths.Right;
import frc.robot.auton.pathfollowing.control.Path;
import frc.robot.commands.auton.RunMotionProfileCommand;
import frc.robot.commands.auton.StartAcquireHatch;
import frc.robot.commands.chassis.TurnInPlace;

public class RSingleHatchRSide extends CommandGroup {
    Path _sidePath = Paths.getPath(Right.TO_RIGHT_CARGO_BAY_FIRST);
    public RSingleHatchRSide() {
        setInterruptible(false);
        addSequential(new RunMotionProfileCommand(_sidePath));
        addSequential(new TurnInPlace(-75, false));
    }
}