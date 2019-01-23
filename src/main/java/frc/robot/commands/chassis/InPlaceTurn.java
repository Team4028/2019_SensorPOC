package frc.robot.commands.chassis;

import edu.wpi.first.wpilibj.command.Command;
import frc.robot.subsystems.Chassis;

public class InPlaceTurn extends Command
{
    private Chassis _chassis = Chassis.getInstance();

    private double _targetAngle;
    private boolean _isTurnRight;
    
    public InPlaceTurn(double targetAngle, boolean isTurnRight)
    {
        _targetAngle = targetAngle;
        _isTurnRight = isTurnRight;
    }
    @Override
    protected void initialize()
    {
        _chassis.setTargetAngleAndTurnDirection(_targetAngle, _isTurnRight);
    }

    @Override
    protected void execute() 
    {
        _chassis.moveToTargetAngle();
    }
    
    @Override
    protected boolean isFinished() 
    { 
        
		return Math.abs(_targetAngle - _chassis.getHeading()) < 2.5; // Returns true when chassis is within angle
                                                                     // deadband
    
    }
    @Override
    protected void end()
    {
        _chassis.stop();
        //_targetAngle = _chassis.get_Heading();
    }
}