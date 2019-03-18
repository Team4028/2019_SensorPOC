package frc.robot.commands.auton.adaptivePaths;


import java.sql.Time;

import com.ctre.phoenix.motorcontrol.ControlMode;

import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj.buttons.Button;
import edu.wpi.first.wpilibj.command.Command;
import frc.robot.sensors.DistanceRev2mSensor;
import frc.robot.sensors.VisionLL;
import frc.robot.sensors.VisionLL.LIMELIGHT_PIPELINE;
import frc.robot.subsystems.Cargo;
import frc.robot.subsystems.Chassis;
import frc.robot.util.GeneralUtilities;

public class BetterVisionPath extends Command
{
    double Pa1, Pa2, dx;
    double kDa1;
    double turnCmd, prevTurnCmd;
    double prevA1, currentTime;
    double startTime;
    Chassis _chassis = Chassis.getInstance();
    VisionLL _limelight = VisionLL.getInstance();
    DistanceRev2mSensor _distanceSensor = DistanceRev2mSensor.getInstance();
    boolean isFirstCycle;
    boolean isForcedFinish;
    Button _a;
    int counter;
    public static final double straightVBus = 0.25;
    public BetterVisionPath(Button a)
    {
        _a=a;
        setInterruptible(true);
        requires(_chassis);
    }
    @Override
    protected void initialize() 
    {
        counter=0;
        currentTime = Timer.getFPGATimestamp();
        startTime = Timer.getFPGATimestamp();
        prevA1=0;
        dx=_limelight.get_xOffset();
        Pa1=0.006;
        Pa2=0.0055;
        turnCmd=0;
        prevTurnCmd = 0;
        kDa1=0.001;
        isFirstCycle=true;
        isForcedFinish = false;
        _limelight.changeLimelightPipeline(LIMELIGHT_PIPELINE.CENTER_PNP);
    }

    @Override
    protected void execute() {
        if(Timer.getFPGATimestamp()-startTime>0.5)
        {
            if(_limelight.get_isTargetInFOV())
            {
                double a1, a2, dA1;
                dx = _limelight.get_xOffset()+7;
                a1 = _limelight.getTheta();
                if(!isFirstCycle)
                {
                    dA1 = kDa1*(a1-prevA1)/(Timer.getFPGATimestamp()-currentTime);
                }
                else
                {
                    dA1=0;
                }
                prevA1=a1;
                a2 = Math.atan2(_limelight.get_xOffset()+7,_limelight.get_yOffset())*180/Math.PI;
                if(a2>90)
                {
                    a2-=180;
                }
                if(a2<-90)
                {
                    a2+=180;
                }
                turnCmd = (Pa2*a2+Pa1*a1);
                System.out.print("A1: "+GeneralUtilities.roundDouble(a1*Pa1, 4));
               // System.out.print(" DA1: "+GeneralUtilities.roundDouble(dA1, 4));
                System.out.print(" A2: " + GeneralUtilities.roundDouble(a2*Pa2, 4));
                System.out.print(" dx: " + GeneralUtilities.roundDouble(dx, 4));
                System.out.println(" Turn: "+ GeneralUtilities.roundDouble(turnCmd, 4));
                _chassis.setLeftRightCommand(ControlMode.PercentOutput, straightVBus+turnCmd, straightVBus-turnCmd);
                prevTurnCmd = turnCmd;
            }
            else
            {
                System.out.println("Lost LL");
                if(Math.abs(turnCmd)<0.04 && !isFirstCycle)
                {
                    _chassis.setLeftRightCommand(ControlMode.PercentOutput, straightVBus, straightVBus);
                }
                else if(isFirstCycle)
                {
                    System.out.println("Cannot See Target");
                    isForcedFinish = true;
                }
                else if(!isFirstCycle && Math.abs(turnCmd)>=0.04)
                {
                    System.out.println("Angle of Approach is Too Aggressive. Also, Ciarn Says You Are Bad and I am Right - Nick :P <3");
                    isForcedFinish = true;
                }
            }
            if(isFirstCycle)
            {
                isFirstCycle = false;
            }
            currentTime = Timer.getFPGATimestamp();
        }

    }
    @Override
    protected boolean isFinished() {
        boolean isFin =  ((!_a.get()) || (_distanceSensor.get_distanceToTargetInInches()>0 && _distanceSensor.get_distanceToTargetInInches()<28)|| isForcedFinish);
        if(isFin)
        {
            System.out.print("Button: "+ !_a.get());
            System.out.print(" Distance: " + (_distanceSensor.get_distanceToTargetInInches()>0 && _distanceSensor.get_distanceToTargetInInches()<22));
            System.out.println(" Force Finished: "+ isForcedFinish);
            _chassis.stop();
            counter++;
            
        }
        else
        {
            counter=0;
        }
        return counter>=4|| Math.abs(turnCmd)>0.4;
    }

    @Override
    protected void end() {
        _chassis.stop();
        System.out.println("END RUNNING");
    }
    @Override
    protected void interrupted() {
        _chassis.stop();
        System.out.println("INTERRUPTED");
    }
}