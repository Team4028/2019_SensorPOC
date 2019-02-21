package frc.robot.commands.elevator;

import edu.wpi.first.wpilibj.command.Command;
import frc.robot.subsystems.Elevator;

public class ZeroElevatorEncoder extends Command {
  Elevator _elevator = Elevator.getInstance();

  public ZeroElevatorEncoder() {
    requires(_elevator);
    setInterruptible(false);
  }

  // Called just before this Command runs the first time
  @Override
  protected void initialize() {}

  // Called repeatedly when this Command is scheduled to run
  @Override
  protected void execute() {
    _elevator.zeroElevatorMotorEncoder();
  }

  // Make this return true when this Command no longer needs to run execute()
  @Override
  protected boolean isFinished() {
    return _elevator.get_hasElevatorBeenZeroed();
  }

  // Called once after isFinished returns true
  @Override
  protected void end() {}

  // Called when another command which requires one or more of the same
  // subsystems is scheduled to run
  @Override
  protected void interrupted() {}
}
