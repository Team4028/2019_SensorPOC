package frc.robot.commands.auton.adaptivePaths;

import com.ctre.phoenix.motorcontrol.ControlMode;

import edu.wpi.first.wpilibj.command.Command;
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
    double counter;

    public FindTarget(SCORING_TARGET target, SIDE side) {
        setInterruptible(true);
        requires(_chassis);
        _scoringTarget = target;
        _side = side;

    }                

    @Override
    protected void initialize() {
        counter = 0;
        double targetAngle = _navX.getTargetAngle(_scoringTarget, _side);
        deltaTheta=_navX.getYaw() - targetAngle;
        System.out.println("deltaTheta: " + deltaTheta);
    }

    @Override
    protected void execute() {
        _chassis.setLeftRightCommand(ControlMode.PercentOutput, -1 * Math.copySign(0.2, deltaTheta), Math.copySign(0.2, deltaTheta));        
        if(_limeLight.get_isTargetInFOV()) {
            counter++;
        } else {
            counter = 0;
        }
    }

    @Override
    protected boolean isFinished() {
        System.out.println("counter"+counter);
        System.out.print(_limeLight.get_isTargetInFOV());
        return counter>=3;
    }

    @Override
    protected void end() {
        _chassis.stop();
    }
}