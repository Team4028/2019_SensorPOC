package frc.robot.commands.auton.autons;

import edu.wpi.first.wpilibj.command.CommandGroup;
import frc.robot.commands.chassis.DriveWithControllers;
import frc.robot.commands.chassis.StopChassis;

public class LineCross extends CommandGroup {
    public LineCross() {
        addSequential(new DriveWithControllers(0.5, 0),1.5);
        addSequential(new StopChassis());
    }
}