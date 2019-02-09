package frc.robot.commands.auton;

import frc.robot.auton.path_planning.problem;
import frc.robot.auton.pathfollowing.RobotState;
import frc.robot.auton.pathfollowing.Paths.Center;
import frc.robot.auton.pathfollowing.control.Path;
import frc.robot.subsystems.Chassis;

import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj.command.Command;

public class Auton_RunProfileFromVision extends Command
{
    Chassis _chassis = Chassis.getInstance();
    private Path _path;
    private double _startTime;

    public Auton_RunProfileFromVision()
    {
        requires(_chassis);
    }

    @Override
    protected void initialize() {
        _path = problem._path;
        RobotState.getInstance().reset(Timer.getFPGATimestamp(), _path.getStartPose());
		_chassis.setWantDrivePath(_path, _path.isReversed());
		//_chassis.setHighGear(true);
		_startTime = Timer.getFPGATimestamp();
    }
    @Override
    protected void execute() {
        if(Timer.getFPGATimestamp() - _startTime > 0.25) {
            if(_chassis.getLeftPos() == 0 || _chassis.getRightPos() == 0) 
            {
                System.out.println(_chassis.getLeftPos());
                System.out.println(_chassis.getRightPos());
				_chassis.forceDoneWithPath();
				System.out.println("Attention Idiots: You Morons Forgot to Plug in The Encoder");
			}
		}
    }
    @Override
    protected boolean isFinished() {
        return _chassis.isDoneWithPath();
    }
    @Override
    protected void end() {
        _chassis.stop();
    }

}