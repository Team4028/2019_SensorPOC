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
    public BetterVisionPath()
    {
        setInterruptible(true);
        requires(_chassis);
    }
    @Override
    protected void initialize() 
    {
        //dx=...;
        Pa1=0.007-0.005*dx;
        Pa2=0.001;
        turnCmd=0;
        prevTurnCmd = 0;
    }

    @Override
    protected void execute() {
        double a1, a2;
       // dx = ...;
        a1 = _limelight.get_angle1InDegrees()*Pa1;
        //a2 = ...;
        //turnCmd = 0.7*prevTurnCmd + 0.3*(Pa2*a2+Pa1*a1);
        System.out.print("A1: "+GeneralUtilities.roundDouble(a1, 4));
        //System.out.print(" A2: " + GeneralUtilities.roundDouble(a2, 4));
        System.out.print(" dx: " + GeneralUtilities.roundDouble(dx, 4));
       // System.out.println(" Turn: "+ GeneralUtilities.roundDouble(turnCmd, 4));
        //_chassis.setLeftRightCommand(ControlMode.PercentOutput, 0.15+turnCmd, 0.15-turnCmd);
        prevTurnCmd = turnCmd;

    }
    @Override
    protected boolean isFinished() {
        return _distanceSensor.get_distanceToTargetInInches()>0 && _distanceSensor.get_distanceToTargetInInches()<10;
    }

    @Override
    protected void end() {
        _chassis.stop();
    }
}