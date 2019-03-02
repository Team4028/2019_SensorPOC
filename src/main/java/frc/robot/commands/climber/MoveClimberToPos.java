package frc.robot.commands.climber;

import edu.wpi.first.wpilibj.command.Command;
import edu.wpi.first.wpilibj.command.CommandGroup;
import edu.wpi.first.wpilibj.smartdashboard.SendableBuilder;
import frc.robot.subsystems.Climber;

public class MoveClimberToPos extends Command
{
    double _targetPos;
    Climber _climber = Climber.getInstance();
    public MoveClimberToPos(double targetPos)
    {
        _targetPos=targetPos;
    }

    @Override
    protected void initialize() {
        
    }
    @Override
    protected void execute() {
        //_climber.
    }
    @Override
    protected boolean isFinished() {
        return false;
    }
}