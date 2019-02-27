package frc.robot.commands.auton.adaptivePaths;

import com.ctre.phoenix.motorcontrol.ControlMode;

import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj.command.Command;
import frc.robot.sensors.GyroNavX;
import frc.robot.sensors.VisionLL;
import frc.robot.subsystems.Chassis;
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

    
    public Auton_turnFromVision() {
        setInterruptible(false);
        requires(_chassis);
        kP=0.045;
        kI=0.03;
        kD=0.05;
        prevTime=Timer.getFPGATimestamp();
        w= 6.283;
        A=(0.01*w-2)/(0.01*w+2);
        B=2/(0.01*w+2);


        
    }

    @Override
    protected void initialize() {
        System.out.print("a: "+A);
        System.out.println(" b: "+ B);
        _canSeeTarget = _limelight.get_isTargetInFOV();
        prevTime=Timer.getFPGATimestamp();
        P=0;
        I=0;
        D=0;   
        error=0;
        prevD=0;
    }

    @Override
    protected void execute() {
        cycleTime= Timer.getFPGATimestamp()-prevTime;
        error = _limelight.getTheta();
        P = kP * error;
        I += kI * error * cycleTime;
        D = (kD*(error-prevError)/cycleTime)*B-A*prevD;
        double output = P+I+D;
        prevD=D;
        System.out.print(" E: " + GeneralUtilities.roundDouble(error, 3));
        System.out.print(" P: "+GeneralUtilities.roundDouble(P, 3));
        System.out.print(" I: "+GeneralUtilities.roundDouble(I, 3));
        System.out.print(" D: "+GeneralUtilities.roundDouble(D, 3));
        System.out.println("Output: " + GeneralUtilities.roundDouble(output, 3));
        prevTime=Timer.getFPGATimestamp();
        prevError=error;
        _chassis.setLeftRightCommand(ControlMode.PercentOutput, output, -output);
    }
    
    @Override
    protected boolean isFinished() {   
        return (Math.abs(error)<2 &&Math.abs(D*cycleTime/kD)< 0.05)|| !_canSeeTarget;                               // deadband
    
    }

    @Override
    protected void end() {
        _chassis.stop();
        System.out.println("Turn Completed");
        System.out.println("Can See Target:" + _canSeeTarget);
        
    }
}