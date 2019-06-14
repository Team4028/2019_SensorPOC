package frc.robot.commands.chassis;

import edu.wpi.first.wpilibj.command.Command;
import frc.robot.subsystems.NEOChassis;

public class StopChassis extends Command
{
    NEOChassis _chassis = NEOChassis.getInstance();
    public StopChassis()
    {
        requires(_chassis);
        setInterruptible(false);
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