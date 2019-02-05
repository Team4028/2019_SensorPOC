/*----------------------------------------------------------------------------*/
/* Copyright (c) 2018 FIRST. All Rights Reserved.                             */
/* Open Source Software - may be modified and shared by FRC teams. The code   */
/* must be accompanied by the FIRST BSD license file in the root directory of */
/* the project.                                                               */
/*----------------------------------------------------------------------------*/

package frc.robot.commands.infeed;

import edu.wpi.first.wpilibj.command.Command;
import frc.robot.subsystems.Cargo;
import frc.robot.subsystems.Cargo.BEAK_POSITION;
import frc.robot.subsystems.Cargo.PUNCH_POSITION;

public class ScoreHatch extends Command {

  // local working variables
  private final int SLEEP_TIME_IN_MS = 100;

  private long _startTimeInMs = 0;
  private enum SCORE_STEP
  {
    UNDEFINED,
    BEAK_STEP,
    WAIT_STEP,
    PUNCH_STEP
  }
  private SCORE_STEP _currentStep = SCORE_STEP.UNDEFINED;

  private Cargo _cargo = Cargo.getInstance();

  // constructor
  public ScoreHatch() {
    requires(_cargo);
    setInterruptible(false);
  }

  // Called just before this Command runs the first time
  @Override
  protected void initialize() 
  { 
    _currentStep = SCORE_STEP.BEAK_STEP;
  }

  // Called repeatedly when this Command is scheduled to run
  @Override
  protected void execute() 
  {
    if(_currentStep == SCORE_STEP.BEAK_STEP)
    {
      _cargo.setBeak(BEAK_POSITION.CLOSED);
      _currentStep = SCORE_STEP.WAIT_STEP;
      _startTimeInMs = System.nanoTime() / 1000000;
    }
    else if(_currentStep == SCORE_STEP.WAIT_STEP)
    {
      long currentTimeInMs = System.nanoTime() / 1000000;
      long elapsedTimeInMs = currentTimeInMs - _startTimeInMs;
      if (elapsedTimeInMs > SLEEP_TIME_IN_MS)
      {
        _currentStep = SCORE_STEP.PUNCH_STEP;
        _cargo.setPunch(PUNCH_POSITION.OUT);
      }
    }
  }

  // Make this return true when this Command no longer needs to run execute()
  @Override
  protected boolean isFinished() {
    return (_currentStep == SCORE_STEP.PUNCH_STEP);
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
