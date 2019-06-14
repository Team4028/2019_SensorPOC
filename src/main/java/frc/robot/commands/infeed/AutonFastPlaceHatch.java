package frc.robot.commands.infeed;

import edu.wpi.first.wpilibj.command.CommandGroup;
import edu.wpi.first.wpilibj.command.WaitCommand;
import frc.robot.commands.infeed.ToggleBeakOpen;
import frc.robot.commands.infeed.TogglePunch;
import frc.robot.subsystems.Cargo;
import frc.robot.subsystems.NEOChassis;

public class AutonFastPlaceHatch extends CommandGroup
{
    NEOChassis _chassis = NEOChassis.getInstance();
    Cargo _cargo = Cargo.getInstance();
    public AutonFastPlaceHatch()
    {
        addParallel(new ToggleBeakOpen());
        addSequential(new WaitCommand(0.16));
        addSequential(new TogglePunch());
        addSequential(new WaitCommand(0.3));
        addSequential(new TogglePunch());

    }
}