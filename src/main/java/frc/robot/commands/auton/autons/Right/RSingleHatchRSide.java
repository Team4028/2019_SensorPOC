package frc.robot.commands.auton.autons.Right;

import edu.wpi.first.wpilibj.command.CommandGroup;
import frc.robot.auton.pathfollowing.Paths;
import frc.robot.auton.pathfollowing.Paths.Right;
import frc.robot.auton.pathfollowing.control.Path;
import frc.robot.commands.auton.RunMotionProfileCommand;
import frc.robot.commands.auton.adaptivePaths.AutoPlaceHatch;
import frc.robot.sensors.GyroNavX.SCORING_TARGET;
import frc.robot.sensors.GyroNavX.SIDE;

public class RSingleHatchRSide extends CommandGroup {
    Path _sidePath = Paths.getPath(Right.TO_RIGHT_CARGO_BAY_FIRST);
    public RSingleHatchRSide() {
        setInterruptible(false);
        addSequential(new RunMotionProfileCommand(_sidePath));
        addSequential(new AutoPlaceHatch());
    }
}