package frc.robot.commands.Climber;

import edu.wpi.first.wpilibj.command.Command;
import frc.robot.subsystems.Climber;
import frc.robot.util.BeakXboxController.Thumbstick;

public class LiftClimber extends Command {

  private Climber _climber = Climber.getInstance();
  Thumbstick _thumbstick;

  public LiftClimber(Thumbstick leftstick) {
    requires(_climber);
    _thumbstick = leftstick;
}

  // Called just before this Command runs the first time
  @Override
  protected void initialize() {
  }

  // Called repeatedly when this Command is scheduled to run
  @Override
  protected void execute() {
     double lift = _thumbstick.getY();
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
  }

  // Called when another command which requires one or more of the same
  // subsystems is scheduled to run
  @Override
  protected void interrupted() {
  }
}