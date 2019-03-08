package frc.robot.commands.auton.adaptivePaths;

import com.ctre.phoenix.motorcontrol.ControlMode;

import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj.command.Command;
import frc.robot.sensors.GyroNavX;
import frc.robot.sensors.VisionLL;
import frc.robot.subsystems.Chassis;
import frc.robot.subsystems.Chassis.ChassisState;
import frc.robot.util.GeneralUtilities;

public class Auton_turnFromVision extends Command { 
    private Chassis _chassis = Chassis.getInstance();
    GyroNavX _navX = GyroNavX.getInstance();
    VisionLL _limelight = VisionLL.getInstance();
    boolean _canSeeTarget;
    private double P,I,D,kP,kI,kD, error, prevError;
    private double currentTime, prevTime, cycleTime;
    double isFinishedError;
    double w, A ,B;
    double prevD;
    boolean isFirstCycle;
    double count;

    
    public Auton_turnFromVision() {
        setInterruptible(false);
        requires(_chassis);
        kP=0.025;
        kI=0.035;
        kD=0.001;
        prevTime=Timer.getFPGATimestamp();     
    }

    @Override
    protected void initialize() {
        _chassis.setChassisState(ChassisState.PERCENT_VBUS);
        _canSeeTarget = _limelight.get_isTargetInFOV();
        prevTime=Timer.getFPGATimestamp();
        P=0;
        I=0;
        D=0;   
        error=0;
        prevD=0;
        isFirstCycle=true;
        count=0;
    }

    @Override
    protected void execute() {
        cycleTime= Timer.getFPGATimestamp()-prevTime;
        error = _limelight.getTheta();
        // error-=2.5;
        if(Math.abs(error)>5)
        {
            error=Math.copySign(5, error);
            I=0;
        }
        else
        {
            I += kI * error * cycleTime;
        }
        P = kP * error;

        
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
        count++;
        _chassis.setLeftRightCommand(ControlMode.PercentOutput, output, -output);
    }
    
    @Override
    protected boolean isFinished() {   
        return count>5 && ((Math.abs(error)<1 &&Math.abs(D)< 0.005)|| !_canSeeTarget);                               // deadband

    
    }

    @Override
    protected void end() {
        _chassis.setCanSeeTarget(_canSeeTarget);
        _chassis.stop();
        System.out.println("Turn Completed");
        System.out.println("Can See Target:" + _canSeeTarget);
        
    }
}