package frc.robot.commands.auton.adaptivePaths;

import edu.wpi.first.wpilibj.command.CommandGroup;
import edu.wpi.first.wpilibj.command.WaitCommand;
import frc.robot.commands.infeed.ToggleBeakOpen;
import frc.robot.commands.infeed.TogglePunch;
import frc.robot.subsystems.Chassis;

public class AutonFastPlaceHatch extends CommandGroup
{
    Chassis _chassis = Chassis.getInstance();
    public AutonFastPlaceHatch()
    {
        if(_chassis.getIsVisionTargetVisible())
        {
            addParallel(new ToggleBeakOpen());
            addSequential(new WaitCommand(0.16));
            addSequential(new TogglePunch());
        } 
    }
}