package frc.robot.commands.auton;

import frc.robot.auton.pathfollowing.RobotState;
import frc.robot.auton.pathfollowing.control.Path;
import frc.robot.subsystems.Chassis;

import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj.command.Command;

public class RunTimedMotionProfileCommand extends Command
{
    Chassis _chassis = Chassis.getInstance();
    private Path _path;
    private double _startTime;
    double _maxTime;
    RobotState _inst = RobotState.getInstance();

    public RunTimedMotionProfileCommand(Path p, double maxTime)
    {
        requires(_chassis);
        _maxTime = maxTime;
        _path = p;
    }

    @Override
    protected void initialize() {
        System.out.println(_path);
        _inst.reset(Timer.getFPGATimestamp(), _path.getStartPose());
		_chassis.setWantDrivePath(_path, _path.isReversed());
		//_chassis.setHighGear(true);
		_startTime = Timer.getFPGATimestamp();
    }
    @Override
    protected void execute() {
        if(Timer.getFPGATimestamp() - _startTime > 0.25) {
			if(_chassis.getLeftPos() == 0 || _chassis.getRightPos() == 0) {
				_chassis.forceDoneWithPath();
				System.out.println("Attention Idiots: You Morons Forgot to Plug in The Encoder");
			}
		}
    }
    @Override
    protected boolean isFinished() {
        //System.out.println("Does the bloody Motion Profile Comand know how freaking lucky it is to Finish?");
        if (Math.floor(Timer.getFPGATimestamp() * 1000) % 1000 == 0){
            System.out.println("Second gotten to:" + Timer.getFPGATimestamp());
        } if (_chassis.isDoneWithPath() || Timer.getFPGATimestamp()-_startTime>=_maxTime){
            System.out.println("Motion Profile Terminating");
            return true;
        } else {
            return false;
        }
    }
    @Override
    protected void end() {
        System.out.println("Motion Profile Properly Terminated");
        System.out.println("Final cAngle: " + _chassis.getHeading());
        _chassis.stop();
    }

}