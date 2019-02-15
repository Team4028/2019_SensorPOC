// package frc.robot.commands.elevator;

// import edu.wpi.first.wpilibj.command.Command;
// import frc.robot.subsystems.Elevator;
// import frc.robot.subsystems.Elevator.ELEVATOR_TARGET_POSITION;

// public class MoveToPresetPosition extends Command {
//   Elevator _elevator = Elevator.getInstance();
//   ELEVATOR_TARGET_POSITION _presetPosition;

//   public MoveToPresetPosition(ELEVATOR_TARGET_POSITION presetPosition) {
//     requires(_elevator);
//     setInterruptible(true);
//     _presetPosition = presetPosition;
//   }

//   // Called just before this Command runs the first time
//   @Override
//   protected void initialize() {}

//   // Called repeatedly when this Command is scheduled to run
//   @Override
//   protected void execute() {
//     _elevator.MoveToPresetPosition(_presetPosition);
//   }

//   // Make this return true when this Command no longer needs to run execute()
//   @Override
//   protected boolean isFinished() {
//     return _elevator.get_isElevatorAtTargetPos();
//   }

//   // Called once after isFinished returns true
//   @Override
//   protected void end() {}

//   // Called when another command which requires one or more of the same
//   // subsystems is scheduled to run
//   @Override
//   protected void interrupted() {}
// }
