package frc.robot.commands.auton;

import edu.wpi.first.wpilibj.command.Command;
import frc.robot.auton.path_planning.problem;
import frc.robot.auton.pathfollowing.RobotState;
import frc.robot.auton.pathfollowing.motion.RigidTransform;
import frc.robot.sensors.GyroNavX;
import frc.robot.subsystems.Chassis;

public class Auton_turnFromVision extends Command
{
    private Chassis _chassis = Chassis.getInstance();
    GyroNavX _navX = GyroNavX.getInstance();

    private double _targetAngle;

    private int latencyCycles = 0;

    boolean isLegit;
    
    public Auton_turnFromVision(){}

    @Override
    protected void initialize()
    {
        if (problem._theta >= 0){
            _chassis.setTargetAngleAndTurnDirection(problem._theta, problem._theta > _navX.getYaw());
        } else {
            _chassis.setTargetAngleAndTurnDirection(360 + problem._theta, problem._theta > _navX.getYaw());
        }
    }

    @Override
    protected void execute() 
    {
        _chassis.moveToTargetAngle();
        latencyCycles++;
        // System.out.println("Heading"+_chassis.getHeading());
        // System.out.println("Error"+(_targetAngle-_chassis.getHeading()));
    }
    
    @Override
    protected boolean isFinished() 
    { 
        
        if((_chassis._angleError < 2.5) && (latencyCycles > 10))
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