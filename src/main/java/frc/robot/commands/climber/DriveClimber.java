package frc.robot.commands.climber;

import edu.wpi.first.wpilibj.command.Command;
import frc.robot.subsystems.Climber;
import frc.robot.util.BeakXboxController.Thumbstick;
import frc.robot.util.BeakXboxController.Trigger;

public class DriveClimber extends Command {

  private Climber _climber = Climber.getInstance();
  Thumbstick _thumbstick;
  double _throttle;
  boolean isAuton;

  public DriveClimber(Thumbstick rightstick) {
    requires(_climber);
    setInterruptible(true);
    _thumbstick = rightstick;
    isAuton = false;
}
  public DriveClimber(double throttle)
  {
    setInterruptible(true);
    requires(_climber);
    _throttle=throttle;
    isAuton = true;
  }

  // Called just before this Command runs the first time
  @Override
  protected void initialize() {
  }

  // Called repeatedly when this Command is scheduled to run
  @Override
  protected void execute() {
    double drive;
    if(isAuton)
    {
      drive=-_throttle;
    }
    else{
      drive= -_thumbstick.getY();
    }
       
    _climber.driveClimber(drive);
  }

  // Make this return true when this Command no longer needs to run execute()
  @Override
  protected boolean isFinished() {
    return true;
  }

  // Called once after isFinished returns true
  @Override
  protected void interrupted() {
    _climber.driveClimber(0);
  }

  // Called when another command which requires one or more of the same
  // subsystems is scheduled to run

}