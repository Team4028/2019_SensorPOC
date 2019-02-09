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
    boolean seenFirstCycle = false;
    boolean seenSecondCycle = false;
    boolean seenThirdCycle = false;
    double deltaTheta;
    double counter = 0;



    public FindTarget(SCORING_TARGET target, SIDE side) 
    {
        _scoringTarget = target;
        _side = side;
    }                

    @Override
    protected void initialize() 
    {
        double targetAngle = GyroNavX.getTargetAngle(_scoringTarget, _side);
        deltaTheta=_navX.getYaw() - targetAngle;
        System.out.println("Target Angle: " + problem._targetAngle);
        System.out.println("deltaTheta: " + deltaTheta);
    }

    @Override
    protected void execute() 
    {
        _chassis.setLeftRightCommand(ControlMode.PercentOutput, -1 * Math.copySign(0.2, deltaTheta), Math.copySign(0.2, deltaTheta));
        System.out.println(Math.copySign(0.2, deltaTheta));
        if(_limeLight.get_isTargetInFOV())
        {
            counter++;
        }
        else
        {
            counter = 0;
        }

    }

    @Override
    protected boolean isFinished() {
        return counter>=3;
        
    }
    @Override
    protected void end() 
    {
        System.out.println("LADIES AND GENTLEMAN... WE'VE GOT EM");
        _chassis.stop();
    }

}