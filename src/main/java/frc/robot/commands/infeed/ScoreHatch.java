/*----------------------------------------------------------------------------*/
/* Copyright (c) 2018 FIRST. All Rights Reserved.                             */
/* Open Source Software - may be modified and shared by FRC teams. The code   */
/* must be accompanied by the FIRST BSD license file in the root directory of */
/* the project.                                                               */
/*----------------------------------------------------------------------------*/

package frc.robot.commands.infeed;

import edu.wpi.first.wpilibj.command.Command;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import frc.robot.subsystems.Cargo;
import frc.robot.subsystems.Cargo.BEAK_OPENCLOSE_POSITION;
import frc.robot.subsystems.Cargo.PUNCH_POSITION;

public class ScoreHatch extends Command {

  // local working variables
  private final int SLEEP_TIME_IN_MS = 0;
  private final int SLEEP_2_TIME_IN_MS = 500;

  private long _startTimeInMs = 0;
  private enum SCORE_STEP {
    UNDEFINED,
    BEAK_CLOSE_STEP,
    WAIT_STEP,
    PUNCH_OUT_STEP,
    WAIT_PUNCH_IN_STEP,
    PUNCH_IN_STEP, 
    FINISHED_STEP
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
  protected void initialize() { 
    _currentStep = SCORE_STEP.BEAK_CLOSE_STEP;
  }

  // Called repeatedly when this Command is scheduled to run
  @Override
  protected void execute() {
    if(_currentStep == SCORE_STEP.BEAK_CLOSE_STEP) {
      _cargo.setBeakOpenClose(BEAK_OPENCLOSE_POSITION.CLOSED);
      _currentStep = SCORE_STEP.WAIT_STEP;
      _startTimeInMs = System.nanoTime() / 1000000;
    }
    else if(_currentStep == SCORE_STEP.WAIT_STEP) {
      long currentTimeInMs = System.nanoTime() / 1000000;
      long elapsedTimeInMs = currentTimeInMs - _startTimeInMs;
      if (elapsedTimeInMs > SLEEP_TIME_IN_MS) {
        _currentStep = SCORE_STEP.PUNCH_OUT_STEP;
      }
    }
    else if (_currentStep == SCORE_STEP.PUNCH_OUT_STEP) {
      _cargo.setPunch(PUNCH_POSITION.OUT);
      _currentStep = SCORE_STEP.WAIT_PUNCH_IN_STEP;
      _startTimeInMs = System.nanoTime() / 1000000;
    }
    else if(_currentStep == SCORE_STEP.WAIT_PUNCH_IN_STEP) {
      long currentTimeInMs = System.nanoTime() / 1000000;
      long elapsedTimeInMs = currentTimeInMs - _startTimeInMs;
      if (elapsedTimeInMs > SLEEP_2_TIME_IN_MS) {
        _currentStep = SCORE_STEP.PUNCH_IN_STEP;
      }
    }
    else if(_currentStep == SCORE_STEP.PUNCH_IN_STEP) {
      _cargo.setPunch(PUNCH_POSITION.IN);
      _currentStep = SCORE_STEP.FINISHED_STEP;
    }
    SmartDashboard.putString("CargoCMD:State", _currentStep.toString());
  }

  // Make this return true when this Command no longer needs to run execute()
  @Override
  protected boolean isFinished() {
    return (_currentStep == SCORE_STEP.FINISHED_STEP);
  }

  // Called once after isFinished returns true
  @Override
  protected void end() {}

  // Called when another command which requires one or more of the same
  // subsystems is scheduled to run
  @Override
  protected void interrupted() {}
}
