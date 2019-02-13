package frc.robot.commands.auton;

import edu.wpi.first.wpilibj.command.Command;
import frc.robot.sensors.GyroNavX;
import frc.robot.sensors.VisionLL;
import frc.robot.sensors.GyroNavX.SCORING_TARGET;
import frc.robot.sensors.GyroNavX.SIDE;
import frc.robot.subsystems.Chassis;

public class Auton_turnFromVision extends Command
{
    private Chassis _chassis = Chassis.getInstance();
    GyroNavX _navX = GyroNavX.getInstance();
    private int latencyCycles;
    boolean isLegit;
    SCORING_TARGET _target;
    SIDE _side;
    
    public Auton_turnFromVision(SCORING_TARGET target, SIDE side){
        _target = target;
        _side = side;
        setInterruptible(false);
        requires(_chassis);
    }

    @Override
    protected void initialize()
    {
        latencyCycles = 0;
        double a1 = VisionLL.getInstance().get_angle1InDegrees()*Math.PI/180;
        double targetAngle = GyroNavX.getTargetAngle(_target, _side);
        double a2 = _navX.getTargetAngle(_target,_side);
        double H = GyroNavX.getInstance().getYaw()*Math.PI/180;
        double l = VisionLL.getInstance().get_distanceToTargetInInches();
        double unsignedTheta = Math.abs(180/Math.PI*(Math.atan2(l*Math.sin(a1 + H),l*Math.cos(a1 + H))));
        double angle;
        if(!(targetAngle==0))
        {
            angle = Math.copySign(unsignedTheta, targetAngle);
        }
        else
        {
            angle = Math.copySign(unsignedTheta, a2);
            System.out.println(a2);
            System.out.println(angle);
        }

        System.out.println("ANGLE: " + angle);
        if (angle >= 0)
        {
            _chassis.setTargetAngleAndTurnDirection(angle, angle > _navX.getYaw());
        } 
        else 
        {
            _chassis.setTargetAngleAndTurnDirection(360 + angle, angle > _navX.getYaw());
        }
    }

    @Override
    protected void execute() 
    {
        _chassis.moveToTargetAngle();
        latencyCycles++;
    }
    
    @Override
    protected boolean isFinished() 
    {   
        if((Math.abs(_chassis._angleError) < 5) && (latencyCycles > 10))
        {
            System.out.println("Done");
            return true;
        } // Returns true when chassis is within angle
        else
        {
            return false;
        }                                // deadband
    
    }
    @Override
    protected void end()
    {
        _chassis.stop();
        System.out.println("chassis stopped");
    }
}