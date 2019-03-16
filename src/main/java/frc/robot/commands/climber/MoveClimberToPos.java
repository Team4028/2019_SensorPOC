package frc.robot.commands.climber;

import edu.wpi.first.wpilibj.command.Command;
import edu.wpi.first.wpilibj.command.CommandGroup;
import frc.robot.subsystems.Climber;

public class MoveClimberToPos extends Command
{
    double _targetPos;
    Climber _climber = Climber.getInstance();
    public MoveClimberToPos(double targetPos)
    {
        setInterruptible(true);
        _targetPos=targetPos;
    }

    @Override
    protected void initialize() {
        
    }
    @Override
    protected void execute() {
        if(_climber.getNativeUnits()>_targetPos)
        {
            _climber.liftClimber(-0.7);
        }
        else
        {
            _climber.liftClimber(0.4);
        }

    }
    @Override
    protected boolean isFinished() {
        return Math.abs(_climber.getNativeUnits()-_targetPos)<200;
    }
    @Override
    protected void end() {
        System.out.println("Ending Move to Preset Pos");
        _climber.liftClimber(0);
    }
}