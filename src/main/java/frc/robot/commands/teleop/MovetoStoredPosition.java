package frc.robot.commands.teleop;

import edu.wpi.first.wpilibj.command.Command;
import frc.robot.sensors.VisionLL;
import frc.robot.subsystems.Cargo;
import frc.robot.subsystems.Elevator;
import frc.robot.subsystems.Elevator.ELEVATOR_TARGET_POSITION;

public class MovetoStoredPosition extends Command {
  Elevator _elevator = Elevator.getInstance();
  Cargo _cargo = Cargo.getInstance();
  VisionLL _vision = VisionLL.getInstance();

  
  public MovetoStoredPosition() {
    requires(_elevator);
    setInterruptible(true);
  }

  // Called just before this Command runs the first time
  @Override
  protected void initialize() {
    _elevator.setTargetPosition(_elevator.getStoredTargetPosition(), _cargo.get_isBeakOpen());
  }

  // Called repeatedly when this Command is scheduled to run
  @Override
  protected void execute() {
    long startTimeInMs = 0;
    //System.out.println("running command");
      if(_cargo.get_isBucketExtended()){
        long currentTimeInMs = System.nanoTime() / 1000000;
        long elapsedTimeInMs = currentTimeInMs - startTimeInMs;
        if(elapsedTimeInMs > 500){
          _elevator.moveToPresetPosition();
          System.out.println("Elevator Move");
        }
      } else {
        _cargo.toggleBucket();
        startTimeInMs = System.nanoTime() / 1000000;
      }
    
  }

  // Make this return true when this Command no longer needs to run execute()
  @Override
  protected boolean isFinished() {
    //System.out.println(_elevator.get_isElevatorAtTargetPos());
    return _elevator.get_isElevatorAtTargetPos();// || _vision.isInVisionMode();
  }

  // Called once after isFinished returns true
  @Override
  protected void end() {}

  // Called when another command which requires one or more of the same
  // subsystems is scheduled to run
  @Override
  protected void interrupted() {}
}
