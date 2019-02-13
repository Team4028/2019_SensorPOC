/*----------------------------------------------------------------------------*/
/* Copyright (c) 2018 FIRST. All Rights Reserved.                             */
/* Open Source Software - may be modified and shared by FRC teams. The code   */
/* must be accompanied by the FIRST BSD license file in the root directory of */
/* the project.                                                               */
/*----------------------------------------------------------------------------*/

package frc.robot.commands.elevator;

import edu.wpi.first.wpilibj.command.Command;
import frc.robot.subsystems.Elevator;
import frc.robot.subsystems.Elevator.ELEVATOR_UP_OR_DOWN;

public class MoveElevator extends Command {

    private ELEVATOR_UP_OR_DOWN _elevatorUpOrDown;
  private Elevator _elevator = Elevator.getInstance();

  public MoveElevator(ELEVATOR_UP_OR_DOWN elevatorUpOrDown){
    requires(_elevator);
    setInterruptible(true);
    _elevatorUpOrDown = elevatorUpOrDown;

}

  // Called just before this Command runs the first time
  @Override
  protected void initialize() {
  }

  // Called repeatedly when this Command is scheduled to run
  @Override
  protected void execute() {
    _elevator.moveElevator(_elevatorUpOrDown);
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
