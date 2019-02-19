package frc.robot.commands.chassis;

import edu.wpi.first.wpilibj.command.Command;
import frc.robot.subsystems.Chassis;
import frc.robot.util.BeakXboxController.Thumbstick;

public class DriveWithControllers extends Command {

  private Chassis _chassis = Chassis.getInstance();
  private Thumbstick _leftThumbstick;
  private Thumbstick _rightThumbstick;
  private boolean isAuton;
  private double _leftCmd, _rightCmd;
  private double previousCmd;
  private double previousRightCmd;
  private double maxAccel, maxDecel;

  public DriveWithControllers(Thumbstick leftThumbstick, Thumbstick righThumbstick) {
    requires(_chassis);
    setInterruptible(true);
    _leftThumbstick = leftThumbstick;
    _rightThumbstick = righThumbstick;
    isAuton=false;
  }

  public DriveWithControllers(double throttle, double turn) {
    requires(_chassis);
    setInterruptible(true);
    isAuton = true;
    _leftCmd=throttle;
    _rightCmd=turn;
  }

  // Called just before this Command runs the first time
  @Override
  protected void initialize() 
  {
    previousCmd=0;
    maxAccel=0.04;
    maxDecel=0.04;
  }

  // Called repeatedly when this Command is scheduled to run
  @Override
  protected void execute() 
  {
    if(!isAuton) {
      _chassis.arcadeDrive(_leftThumbstick.getY(), _rightThumbstick.getX());
      // if(_leftThumbstick.getY()-previousCmd>0)
      // {
      //   _chassis.arcadeDrive(Math.min(_leftThumbstick.getY(), previousCmd+maxAccel), _rightThumbstick.getX());
      //   previousCmd=Math.min(_leftThumbstick.getY(), previousCmd+maxAccel);
      // }
      // else
      // {
      //   _chassis.arcadeDrive(Math.max(_leftThumbstick.getY(), previousCmd-maxDecel), _rightThumbstick.getX());
      //   previousCmd=Math.max(_leftThumbstick.getY(), previousCmd-maxDecel);
      // }
    } else {
      _chassis.arcadeDrive(_leftCmd, _rightCmd);
    }
  }

  // Make this return true when this Command no longer needs to run execute()
  @Override
  protected boolean isFinished() {
    return false;
  }

  // Called once after isFinished returns true
  @Override
  protected void end() 
  {
    _chassis.stop();
  }

  // Called when another command which requires one or more of the same
  // subsystems is scheduled to run
  @Override
  protected void interrupted() 
  {}
}
