package frc.robot.commands.auton.autons;

import edu.wpi.first.wpilibj.command.CommandGroup;
import frc.robot.commands.chassis.InPlaceTurn;

public class TurnTest extends CommandGroup
{
    public TurnTest()
    {
        addSequential(new InPlaceTurn(90, true));
    }
}