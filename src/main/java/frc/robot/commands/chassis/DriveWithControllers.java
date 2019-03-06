package frc.robot.commands.chassis;

import edu.wpi.first.wpilibj.command.Command;
import frc.robot.subsystems.Chassis;
import frc.robot.util.BeakXboxController.Thumbstick;

public class DriveWithControllers extends Command {

  private Chassis _chassis = Chassis.getInstance();
  private Thumbstick _leftThumbstick;
  private Thumbstick _rightThumbstick;
  private boolean isAuton;
  private double _throttleCmd, _turnCmd;


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
    _throttleCmd=throttle;
    _turnCmd=turn;
  }

  // Called just before this Command runs the first time
  @Override
  protected void initialize() {}

  // Called repeatedly when this Command is scheduled to run
  @Override
  protected void execute() 
  {
    if(!isAuton) {
      _chassis.arcadeDrive(_leftThumbstick.getY(), _rightThumbstick.getX());
    } else {
      _chassis.arcadeDrive(_throttleCmd, _turnCmd);
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
