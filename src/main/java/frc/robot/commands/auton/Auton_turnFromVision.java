package frc.robot.commands.auton;

import edu.wpi.first.wpilibj.command.Command;
import frc.robot.auton.path_planning.problem;
import frc.robot.auton.pathfollowing.RobotState;
import frc.robot.auton.pathfollowing.motion.RigidTransform;

import frc.robot.subsystems.Chassis;

public class Auton_turnFromVision extends Command
{
    private Chassis _chassis = Chassis.getInstance();

    private double _targetAngle;

    boolean isLegit;
    
    public Auton_turnFromVision(){}

    @Override
    protected void initialize()
    {
        _targetAngle = problem._theta;
        double dTheta = _targetAngle -  RobotState.getInstance().getLatestFieldToVehicle().getValue().getRotation().getDegrees();
        if (dTheta == 0){
            isLegit = false;
        } else if (dTheta < 0) {
            isLegit = true;
            _chassis.setTargetAngleAndTurnDirection(_targetAngle, false);
        } else {
            isLegit = true;
            _chassis.setTargetAngleAndTurnDirection(_targetAngle, true);
        }
    }

    @Override
    protected void execute() 
    {
        if (isLegit){
            _chassis.moveToTargetAngle();
        }
        // System.out.println("Heading"+_chassis.getHeading());
        // System.out.println("Error"+(_targetAngle-_chassis.getHeading()));
    }
    
    @Override
    protected boolean isFinished() 
    { 
        
        if(_chassis._angleError < 5)
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