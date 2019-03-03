package frc.robot.commands.climber;

import edu.wpi.first.wpilibj.command.Command;
import frc.robot.subsystems.Climber;

public class ZeroClimber extends Command
{
    Climber _climber = Climber.getInstance();
    public ZeroClimber()
    {
        setInterruptible(false);
        requires(_climber);
    }
    @Override
    protected void execute() {
        _climber.liftClimber(0.1);
    }
    @Override
    protected boolean isFinished() {
        return _climber.isClimberatTop();
    }
    @Override
    protected void end() {
        _climber.zeroClimber();
    }
}