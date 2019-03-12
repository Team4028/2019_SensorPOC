package frc.robot.commands.elevator;

import edu.wpi.first.wpilibj.buttons.Trigger;
import edu.wpi.first.wpilibj.command.Command;
import frc.robot.sensors.VisionLL;
import frc.robot.subsystems.Cargo;
import frc.robot.subsystems.Elevator;
import frc.robot.subsystems.Elevator.ELEVATOR_TARGET_POSITION;

public class MoveToPresetPosition extends Command {
  Elevator _elevator = Elevator.getInstance();
  Cargo _cargo = Cargo.getInstance();
  VisionLL _vision = VisionLL.getInstance();
  boolean isTeleop;

  private ELEVATOR_TARGET_POSITION _presetPosition;
  Trigger _rt;
  
  public MoveToPresetPosition(ELEVATOR_TARGET_POSITION presetPosition, Trigger rt) {
    System.out.println("Setting target" + presetPosition);
    requires(_elevator);
    setInterruptible(true);
    _presetPosition = presetPosition;
    _rt=rt;
    isTeleop = true;
  }
  public MoveToPresetPosition(ELEVATOR_TARGET_POSITION presetPosition) {
    System.out.println("Setting target" + presetPosition);
    requires(_elevator);
    setInterruptible(true);
    _presetPosition = presetPosition;
    isTeleop = false;
  }
  

  // Called just before this Command runs the first time
  @Override
  protected void initialize() {
      _elevator.setTargetPosition(_presetPosition, _cargo.get_HasHatch());
  }

  // Called repeatedly when this Command is scheduled to run
  @Override
  protected void execute() {
    if(isTeleop)
    {
      if(!_rt.get())
      {
          _elevator.moveToPresetPosition();
      }
      
    }
      
    
    if(!isTeleop)
    {
      System.out.println("running command");
      _elevator.moveToPresetPosition();
      
    }
  }

  // Make this return true when this Command no longer needs to run execute()
  @Override
  protected boolean isFinished() {
    // System.out.println(_elevator.get_isElevatorAtTargetPos());
    // System.out.println((isTeleop&&_rt.get()));
    return _elevator.get_isElevatorAtTargetPos()|| (isTeleop&&_rt.get());// || _vision.isInVisionMode();
  }

  // Called once after isFinished returns true
  @Override
  protected void end() {}

  // Called when another command which requires one or more of the same
  // subsystems is scheduled to run
  @Override
  protected void interrupted() {}
}
