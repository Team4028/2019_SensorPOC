package frc.robot.commands.auton;


import edu.wpi.first.wpilibj.command.CommandGroup;
import edu.wpi.first.wpilibj.command.WaitCommand;
import frc.robot.commands.auton.util.Series_Command;
import frc.robot.commands.auton.util.Simultaneous_Command;
import frc.robot.commands.infeed.AcquireHatch;
import frc.robot.commands.infeed.ToggleBeakInOut;
import frc.robot.commands.infeed.ToggleBeakOpenClose;
import frc.robot.subsystems.Cargo;

public class StartAcquireHatch extends CommandGroup
{
    Cargo _cargo = Cargo.getInstance();
    public StartAcquireHatch()
    {
        setInterruptible(false);
        addSequential(new ToggleBeakOpenClose());
        addSequential(new WaitCommand(0.08));
        addParallel(new ToggleBeakInOut());
        addSequential(new WaitCommand(0.1));
    }
}