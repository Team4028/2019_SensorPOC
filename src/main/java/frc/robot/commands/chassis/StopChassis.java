package frc.robot.commands.chassis;

import edu.wpi.first.wpilibj.command.Command;
import frc.robot.subsystems.Chassis;

public class StopChassis extends Command
{
    Chassis _chassis = Chassis.getInstance();
    public StopChassis()
    {
        requires(_chassis);
        setInterruptible(true);
    }
    @Override
    protected void initialize() {
        _chassis.stop();
    }
    @Override
    protected boolean isFinished() {
        return true;
    }

}