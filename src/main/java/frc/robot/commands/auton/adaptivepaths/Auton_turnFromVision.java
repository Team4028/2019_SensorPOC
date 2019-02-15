package frc.robot.commands.auton.adaptivePaths;

import com.ctre.phoenix.motorcontrol.ControlMode;

import edu.wpi.first.wpilibj.command.Command;
import frc.robot.sensors.GyroNavX;
import frc.robot.sensors.VisionLL;
import frc.robot.subsystems.Chassis;

public class Auton_turnFromVision extends Command { 
    private Chassis _chassis = Chassis.getInstance();
    GyroNavX _navX = GyroNavX.getInstance();
    VisionLL _limelight = VisionLL.getInstance();
    
    public Auton_turnFromVision() {
        setInterruptible(false);
        requires(_chassis);
    }

    @Override
    protected void initialize() {}

    @Override
    protected void execute() {
       if(_limelight.get_angle1InDegrees()>0) {
        _chassis.setLeftRightCommand(ControlMode.PercentOutput, 0.30, -0.30);
       } else {
        _chassis.setLeftRightCommand(ControlMode.PercentOutput, -0.30, 0.30);
       }
    }
    
    @Override
    protected boolean isFinished() {   
        return Math.abs(_limelight.get_angle1InDegrees())< 3;                               // deadband
    
    }

    @Override
    protected void end() {
        _chassis.stop();
        System.out.println("Turn Completed");
    }
}