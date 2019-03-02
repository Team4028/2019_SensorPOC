package frc.robot.commands.elevator;

import edu.wpi.first.wpilibj.command.Command;
import frc.robot.sensors.VisionLL;
import frc.robot.subsystems.Cargo;
import frc.robot.subsystems.Elevator;
import frc.robot.subsystems.Elevator.ELEVATOR_TARGET_POSITION;

public class MoveToPresetPosition extends Command {
  Elevator _elevator = Elevator.getInstance();
  Cargo _cargo = Cargo.getInstance();
  VisionLL _vision = VisionLL.getInstance();
  ELEVATOR_TARGET_POSITION _presetPosition;
  
  private long _startTimeInMs = 0;

  public MoveToPresetPosition(ELEVATOR_TARGET_POSITION presetPosition) {
    requires(_elevator);
    requires(_cargo);
    setInterruptible(true);
    _presetPosition = presetPosition;
    _elevator.setTargetPosition(presetPosition, _cargo.get_HasHatch());
  }

  public MoveToPresetPosition() {
    requires(_elevator);
    requires(_cargo);
    setInterruptible(true);
  }

  // Called just before this Command runs the first time
  @Override
  protected void initialize() {}

  // Called repeatedly when this Command is scheduled to run
  @Override
  protected void execute() {
    long _startTimeInMs = System.nanoTime() / 1000000;
    if(!_vision.isInVisionMode()){
      if(_cargo.get_isBucketExtended()){
        long currentTimeInMs = System.nanoTime() / 1000000;
        long elapsedTimeInMs = currentTimeInMs - _startTimeInMs;
        if(elapsedTimeInMs > 500){
          _elevator.moveToPresetPosition();
        }
      } else {
        _cargo.toggleBucket();
        _startTimeInMs = System.nanoTime() / 1000000;
      }
    }
  }

  // Make this return true when this Command no longer needs to run execute()
  @Override
  protected boolean isFinished() {
    return _elevator.get_isElevatorAtTargetPos() || _vision.isInVisionMode();
  }

  // Called once after isFinished returns true
  @Override
  protected void end() {}

  // Called when another command which requires one or more of the same
  // subsystems is scheduled to run
  @Override
  protected void interrupted() {}
}
