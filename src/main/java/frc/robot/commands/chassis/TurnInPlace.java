package frc.robot.commands.chassis;

import com.ctre.phoenix.motorcontrol.ControlMode;

import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj.command.Command;
import frc.robot.sensors.GyroNavX;
import frc.robot.subsystems.Chassis;
import frc.robot.util.GeneralUtilities;

public class TurnInPlace extends Command {
    private Chassis _chassis = Chassis.getInstance();
    private double P,I,D,kP,kI,kD, error, prevError;
    private double prevTime, cycleTime;
    double prevD;
    boolean isFirstCycle;
    double _targetAngle;
    boolean _isTurnRight;

    
    public TurnInPlace(double targetAngle, boolean isTurnRight) {
        setInterruptible(true);
        requires(_chassis);
        kP=0.019;
        kI=0.0;
        kD=0.005;
        prevTime=Timer.getFPGATimestamp();
        _targetAngle=targetAngle;
        if(_targetAngle>180)
        {
            targetAngle-=360;
        }
        _isTurnRight=isTurnRight;     
    }

    @Override
    protected void initialize() {
        /*
        if(_isTurnRight && (_chassis.getPositiveHeading()-_targetAngle)>0)
        {
            _targetAngle+=360;
        }
        else if((!_isTurnRight) && (_chassis.getPositiveHeading() - _targetAngle)<0)
        {
            _targetAngle-=360;
        }
        prevTime=Timer.getFPGATimestamp();
        P=0;
        I=0;
        D=0;   
        error=0;
        prevD=0;
        isFirstCycle=true;
        */
        _chassis.setTargetAngleAndTurnDirection(_targetAngle, _isTurnRight);
        _chassis.moveToTargetAngle();
    }

    @Override
    protected void execute() {
        /*
        cycleTime= Timer.getFPGATimestamp()-prevTime;
        error = _chassis.getPositiveHeading()-_targetAngle;
        if (error>180)
        {
            error-=360;
        }
        else if(error<-180)
        {
            error+=360;
        }
        if(Math.abs(error)>10)
        {
            error=Math.copySign(10, error);
        }
        P = kP * error;
        I += kI * error * cycleTime;
        if(isFirstCycle)
        {
            D=0;
            isFirstCycle=false;
        }
        else
        {
            D = (kD*(error-prevError)/cycleTime)*0.3+0.7*prevD;           
        }
        double output = P+I+D;
        prevD=D;
        System.out.print(" E: " + GeneralUtilities.roundDouble(error, 3));
        System.out.print(" P: "+GeneralUtilities.roundDouble(P, 3));
        System.out.print(" I: "+GeneralUtilities.roundDouble(I, 3));
        System.out.print(" D: "+GeneralUtilities.roundDouble(D, 3));
        System.out.println("Output: " + GeneralUtilities.roundDouble(output, 3));
        prevTime=Timer.getFPGATimestamp();
        prevError=error;
        _chassis.setLeftRightCommand(ControlMode.PercentOutput, -output, output);
        */
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