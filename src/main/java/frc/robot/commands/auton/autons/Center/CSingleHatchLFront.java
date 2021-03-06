package frc.robot.commands.auton.autons.Center;

import edu.wpi.first.wpilibj.command.CommandGroup;
import frc.robot.auton.pathfollowing.Paths;
import frc.robot.auton.pathfollowing.Paths.Center;
import frc.robot.auton.pathfollowing.control.Path;
import frc.robot.commands.auton.RunMotionProfileCommand;
import frc.robot.commands.auton.StartAcquireHatch;
import frc.robot.commands.auton.adaptivePaths.AutoPlaceHatch;
import frc.robot.commands.auton.util.printTimeFromStart;


public class CSingleHatchLFront extends CommandGroup {
    Path _toCargoShipFront = Paths.getPath(Center.TO_FRONT_CARGO_SHIP_L);
    public CSingleHatchLFront() {
        setInterruptible(false);
        addSequential(new StartAcquireHatch());
        addSequential(new RunMotionProfileCommand(_toCargoShipFront));
        addSequential(new AutoPlaceHatch());
    }
}