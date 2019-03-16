package frc.robot.commands.climber;

import edu.wpi.first.wpilibj.command.Command;
import frc.robot.subsystems.Climber;
import frc.robot.util.BeakXboxController.Thumbstick;

public class LiftClimber extends Command {

  private Climber _climber = Climber.getInstance();
  Thumbstick _thumbstick;
  boolean isAuton;
  double _throttle;

  public LiftClimber(Thumbstick leftstick) {
    requires(_climber);
    setInterruptible(true);
    _thumbstick = leftstick;
    isAuton=false;
}
public LiftClimber(double throttle)
{
  _throttle=throttle;
  setInterruptible(true);
  isAuton=true;
}

  // Called just before this Command runs the first time
  @Override
  protected void initialize() {
  }

  // Called repeatedly when this Command is scheduled to run
  @Override
  protected void execute() {
     double lift;
     if(isAuton)
     {
       lift=_throttle;
     } 
     else
     {
       lift= _thumbstick.getY();
     }

     _climber.liftClimber(lift);
  }

  // Make this return true when this Command no longer needs to run execute()
  @Override
  protected boolean isFinished() {
    return false;
  }

  // Called once after isFinished returns true
  @Override
  protected void end() {
    //_climber.liftClimber(0);
  }

  // Called when another command which requires one or more of the same
  // subsystems is scheduled to run
  @Override
  protected void interrupted() {
  }
}