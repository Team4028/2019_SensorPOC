package frc.robot.commands.chassis;

import edu.wpi.first.wpilibj.command.Command;
import frc.robot.sensors.GyroNavX;
import frc.robot.subsystems.Cargo;
import frc.robot.subsystems.Elevator;
import frc.robot.subsystems.NEOChassis;
import frc.robot.subsystems.Elevator.ELEVATOR_TARGET_POSITION;
import frc.robot.util.BeakXboxController.Thumbstick;

public class DriveWithControllers extends Command {

  private NEOChassis _chassis = NEOChassis.getInstance();
  private Elevator _elevator = Elevator.getInstance();
  private Thumbstick _leftThumbstick;
  private Thumbstick _rightThumbstick;
  private boolean isAuton;
  private double _throttleCmd, _turnCmd;
  private Cargo _cargo = Cargo.getInstance();
  private GyroNavX _navX = GyroNavX.getInstance();


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
    if(Math.abs(_navX.getPitch())<15)
    {
      if(!isAuton) {
        if((_elevator.get_ElevatorPos()>4028))
        {
          _chassis.elevatorUpArcadeDrive(_leftThumbstick.getY(), _rightThumbstick.getX());
        }
        else
        {
          _chassis.arcadeDrive(_leftThumbstick.getY(), _rightThumbstick.getX());
        }
      } else {
        _chassis.arcadeDrive(_throttleCmd, _turnCmd);
      }
    }
    else
    {
      _chassis.stop();
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
