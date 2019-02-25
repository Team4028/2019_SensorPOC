package frc.robot.commands.chassis;

import com.ctre.phoenix.motorcontrol.ControlMode;
import edu.wpi.first.wpilibj.command.Command;
import frc.robot.subsystems.Chassis;

public class TurnInPlace extends Command 
{
    private Chassis _chassis = Chassis.getInstance();
    private double _targetAngle;
    private boolean _isTurnRight;

    
    public TurnInPlace(double targetAngle, boolean isTurnRight) 
    {
        setInterruptible(true);
        requires(_chassis);
        _targetAngle = targetAngle;
        _isTurnRight = isTurnRight;
    }

    @Override
    protected void initialize() {}

    @Override
    protected void execute() 
    {
        
        if(_isTurnRight) 
        {
            _chassis.setLeftRightCommand(ControlMode.PercentOutput, 0.55,-0.55);
        } 
        else 
        {
            _chassis.setLeftRightCommand(ControlMode.PercentOutput, -0.55, 0.55);
        }
    }
    
    @Override
    protected boolean isFinished() { 
        if(!(_targetAngle==0)) {
            return Math.abs(_chassis.getPositiveHeading()-_targetAngle)<6;
        } else {
            return _chassis.getPositiveHeading()>354||_chassis.getPositiveHeading()<6;
        }                             // deadband
    }

    @Override
    protected void end() {
        _chassis.stop();
    }
}