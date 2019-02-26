package frc.robot.commands.Climber;

import edu.wpi.first.wpilibj.command.Command;
import frc.robot.subsystems.Climber;
import frc.robot.util.BeakXboxController.Thumbstick;

public class PositionClimber extends Command {

  private Climber _climber = Climber.getInstance();

  public PositionClimber() {
    requires(_climber);
}

  // Called just before this Command runs the first time
  @Override
  protected void initialize() {
  }

  // Called repeatedly when this Command is scheduled to run
  @Override
  protected void execute() {
      _climber.setClimberPosition();
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