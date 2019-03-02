package frc.robot.commands.auton.adaptivePaths;

import edu.wpi.first.wpilibj.command.CommandGroup;
import edu.wpi.first.wpilibj.command.WaitCommand;
import frc.robot.commands.infeed.ToggleBeakOpen;
import frc.robot.commands.infeed.TogglePunch;

public class AutonFastPlaceHatch extends CommandGroup
{
    public AutonFastPlaceHatch()
    {
        addParallel(new ToggleBeakOpen());
        addSequential(new WaitCommand(0.12));
        addSequential(new TogglePunch());
        
    }
}