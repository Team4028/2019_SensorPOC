package frc.robot.commands.auton.autons;

import edu.wpi.first.wpilibj.command.CommandGroup;
import frc.robot.auton.pathfollowing.Paths;
import frc.robot.auton.pathfollowing.Paths.Left;
import frc.robot.commands.auton.RunTimedMotionProfileCommand;
import frc.robot.commands.chassis.DriveSetDistance;
import frc.robot.commands.chassis.DriveWithControllers;
import frc.robot.commands.chassis.StopChassis;

public class LineCross extends CommandGroup {
    public LineCross() {
        addSequential(new DriveWithControllers(0.5, 0.2),1.5);
    }
}