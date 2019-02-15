package frc.robot.commands.auton.autons;

import edu.wpi.first.wpilibj.command.CommandGroup;
import frc.robot.commands.chassis.DriveWithControllers;

public class LineCross extends CommandGroup
{
    public LineCross()
    {
        addSequential(new DriveWithControllers(0.2, 0.2),2.0);
    }
}