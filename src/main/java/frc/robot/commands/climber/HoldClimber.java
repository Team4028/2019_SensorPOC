package frc.robot.commands.climber;

import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj.command.Command;
import frc.robot.subsystems.Climber;

public class HoldClimber extends Command
{
    double _timeout;
    double startTime;
    Climber _climber = Climber.getInstance();
    boolean onTimer;
    public HoldClimber()
    {
        setInterruptible(true);
        onTimer=false;
    }
    public HoldClimber(double timeout)
    {
        setInterruptible(true);
        _timeout=timeout;
        onTimer=true;

    }
    @Override
    protected void initialize() {
        startTime = Timer.getFPGATimestamp();
    }
    @Override
    protected void execute() {
        _climber.HoldClimber();
    }
    @Override
    protected boolean isFinished() {
        if(onTimer)
        {
            return (Timer.getFPGATimestamp()-startTime)>_timeout;
        }
        return false;
    }
}