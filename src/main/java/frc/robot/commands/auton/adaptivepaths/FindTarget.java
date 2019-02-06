package frc.robot.commands.auton.adaptivepaths;

import com.ctre.phoenix.motorcontrol.ControlMode;

import edu.wpi.first.wpilibj.command.Command;
import edu.wpi.first.wpilibj.interfaces.Gyro;
import frc.robot.auton.path_planning.problem;
import frc.robot.sensors.GyroNavX;
import frc.robot.sensors.VisionLL;
import frc.robot.sensors.GyroNavX.SCORING_TARGET;
import frc.robot.sensors.GyroNavX.SIDE;
import frc.robot.subsystems.Chassis;

public class FindTarget extends Command
{
    double conjecturedTarget;
    GyroNavX _navX = GyroNavX.getInstance();
    VisionLL _limeLight= VisionLL.getInstance();
    SCORING_TARGET _scoringTarget;
    SIDE _side;
    double _targetAngle;
    Chassis _chassis = Chassis.getInstance();

    public FindTarget() 
    {

    }                

    @Override
    protected void initialize() 
    {
        _targetAngle =  problem._targetAngle;
    }

    @Override
    protected void execute() 
    {
        if (_navX.getYaw() > _targetAngle)
        {
            _chassis.setLeftRightCommand(ControlMode.PercentOutput, -0.2, 0.2);
        }
        else
        {
            _chassis.setLeftRightCommand(ControlMode.PercentOutput, 0.2, -0.2);
        }
    }

    @Override
    protected boolean isFinished() {
        return _limeLight.get_isTargetInFOV();
    }
    @Override
    protected void end() 
    {
        System.out.println("LADIES AND GENTLEMAN... WE'VE GOT EM");
        _chassis.stop();
    }

}