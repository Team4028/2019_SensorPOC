package frc.robot.commands.chassis;

import com.ctre.phoenix.motorcontrol.ControlMode;

import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj.command.Command;
import frc.robot.sensors.GyroNavX;
import frc.robot.subsystems.Chassis;
import frc.robot.util.GeneralUtilities;

public class TurnFixedAngle extends Command {
    private Chassis _chassis = Chassis.getInstance();
    GyroNavX _navX = GyroNavX.getInstance();
    double prevD;
    boolean isFirstCycle;
    double _targetAngle;
    boolean _isTurnRight;


    
    public TurnFixedAngle(double turnAngle, boolean isTurnRight) {
        setInterruptible(true);
        requires(_chassis);
        _isTurnRight=isTurnRight;
        _targetAngle=turnAngle;
    }

    @Override
    protected void initialize() {
        boolean isNavXReversed = _navX.getIsReversed();
        double currentAngle = _navX.getYaw();
        double turnAddend;
        _targetAngle+=currentAngle;
        // if ((!isNavXReversed) ^ _isTurnRight){
        //     turnAddend = turnAngle;
        // } else {
        //     turnAddend = -1 * turnAngle;
        // }
        // targetAngle = currentAngle + turnAddend;
        // if(_targetAngle>180)
        // {
        //     targetAngle-=360;
        // }
        _chassis.setTargetAngleAndTurnDirection(_targetAngle, _isTurnRight);
        _chassis.moveToTargetAngle();
    }

    @Override
    protected void execute() {
        _chassis.updateAngleError();
    }
    
    @Override
    protected boolean isFinished() {   
        if(_targetAngle==180||_targetAngle==-180)
		{
			return _chassis.getHeading()>178 || _chassis.getHeading()<-178;	
		}
		else
		{
			return Math.abs(_targetAngle - _chassis.getHeading()) < 2.5;		// Returns true when chassis is within angle deadband
		}
    }

    @Override
    protected void end() {
        _chassis.stop();
        System.out.println("Turn Completed");
        
    }
}