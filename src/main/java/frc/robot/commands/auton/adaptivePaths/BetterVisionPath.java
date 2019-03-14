package frc.robot.commands.auton.adaptivePaths;

import com.ctre.phoenix.motorcontrol.ControlMode;

import edu.wpi.first.wpilibj.command.Command;
import frc.robot.sensors.DistanceRev2mSensor;
import frc.robot.sensors.VisionLL;
import frc.robot.subsystems.Chassis;
import frc.robot.util.GeneralUtilities;

public class BetterVisionPath extends Command
{
    double Pa1, Pa2, dx;
    double turnCmd, prevTurnCmd;
    Chassis _chassis = Chassis.getInstance();
    VisionLL _limelight = VisionLL.getInstance();
    DistanceRev2mSensor _distanceSensor = DistanceRev2mSensor.getInstance();
    boolean isFirstCycle;
    boolean isForcedFinish;
    public static final double straightVBus = 0.15;
    public BetterVisionPath()
    {
        setInterruptible(true);
        requires(_chassis);
    }
    @Override
    protected void initialize() 
    {
        dx=_limelight.get_xOffset();
        Pa1=0.007-0.005*dx;
        Pa2=0.001;
        turnCmd=0;
        prevTurnCmd = 0;
        isFirstCycle=true;
        isForcedFinish = false;
    }

    @Override
    protected void execute() {
        if(_limelight.get_isTargetInFOV())
        {
            double a1, a2;
            dx = _limelight.get_xOffset();
            Pa1=0.007-0.005*dx;
            a1 = _limelight.get_angle1InDegrees()*Pa1;
            a2 = _limelight.g;
            turnCmd = 0.7*prevTurnCmd + 0.3*(Pa2*a2+Pa1*a1);
            System.out.print("A1: "+GeneralUtilities.roundDouble(a1, 4));
            System.out.print(" A2: " + GeneralUtilities.roundDouble(a2, 4));
            System.out.print(" dx: " + GeneralUtilities.roundDouble(dx, 4));
            System.out.println(" Turn: "+ GeneralUtilities.roundDouble(turnCmd, 4));
            _chassis.setLeftRightCommand(ControlMode.PercentOutput, straightVBus+turnCmd, straightVBus-turnCmd);
            prevTurnCmd = turnCmd;
        }
        else
        {
            if(turnCmd<0.1 && !isFirstCycle)
            {
                _chassis.setLeftRightCommand(ControlMode.PercentOutput, straightVBus, straightVBus);
            }
            else if(isFirstCycle)
            {
                System.out.println("Cannot See Target");
                isForcedFinish = true;
            }
            else if(!isFirstCycle && turnCmd>=0.1)
            {
                System.out.println("Angle of Approach is Too Aggressive. Also, Ciarn Says You Are Bad and I am Right - Nick :P <3");
                isForcedFinish = true;
            }
        }
        if(isFirstCycle)
        {
            isFirstCycle = false;
        }

    }
    @Override
    protected boolean isFinished() {
        return (_distanceSensor.get_distanceToTargetInInches()>0 && _distanceSensor.get_distanceToTargetInInches()<10)|| isForcedFinish;
    }

    @Override
    protected void end() {
        _chassis.stop();
    }
}