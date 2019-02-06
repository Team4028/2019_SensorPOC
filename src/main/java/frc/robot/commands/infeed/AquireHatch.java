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
import frc.robot.subsystems.Cargo.MECHANISM_POSITION;
import frc.robot.subsystems.Cargo.PUNCH_POSITION;

public class AquireHatch extends Command 
{
  //Local Variables
  private final int BEAK_CLOSE_TO_PUNCH_WAIT_TIME = 20;
  private final int PUNCH_IN_EXTEND_MECH_WAIT_TIME = 40;
  private final int EXTEND_OUT_TO_BEAK_OPEN = 100;  

  private long _startTimeInMs = 0;
  private enum AQUIRE_HATCH_STEP
  {
    UNDEFINED,
    BEAK_CLOSE_STEP,
    WAIT_TO_PUNCH,
    PUNCH_IN_STEP,
    WAIT_TO_EXTEND,
    EXTEND_MECH_STEP,
    WAIT_TO_OPEN,
    BEAK_OPEN_STEP
  }
  private AQUIRE_HATCH_STEP _currentStep = AQUIRE_HATCH_STEP.UNDEFINED;


  private Cargo _cargo = Cargo.getInstance();
  public AquireHatch() 
  {
    setInterruptible(false);
    requires(_cargo);
  }

  // Called just before this Command runs the first time
  @Override
  protected void initialize() 
  {
    _currentStep = AQUIRE_HATCH_STEP.BEAK_CLOSE_STEP;
  }

  // Called repeatedly when this Command is scheduled to run
  @Override
  protected void execute() 
  {
    if(_currentStep == AQUIRE_HATCH_STEP.BEAK_CLOSE_STEP)
    {
      _cargo.setBeak(BEAK_POSITION.CLOSED);
      _currentStep = AQUIRE_HATCH_STEP.WAIT_TO_PUNCH;
      _startTimeInMs = System.nanoTime() / 1000000;
    }
    else if(_currentStep == AQUIRE_HATCH_STEP.WAIT_TO_PUNCH)
    {
      long currentTimeInMs = System.nanoTime() / 1000000;
      long elapsedTimeInMs = currentTimeInMs - _startTimeInMs;
      if (elapsedTimeInMs > BEAK_CLOSE_TO_PUNCH_WAIT_TIME)
      {
        _currentStep = AQUIRE_HATCH_STEP.PUNCH_IN_STEP;
      }
    }
    else if(_currentStep == AQUIRE_HATCH_STEP.PUNCH_IN_STEP)
    {
      _cargo.setPunch(PUNCH_POSITION.IN);
      _currentStep = AQUIRE_HATCH_STEP.WAIT_TO_EXTEND;
      _startTimeInMs = System.nanoTime() / 1000000;
    }
    else if(_currentStep == AQUIRE_HATCH_STEP.WAIT_TO_EXTEND)
    {
      long currentTimeInMs = System.nanoTime() / 1000000;
      long elapsedTimeInMs = currentTimeInMs - _startTimeInMs;
      if (elapsedTimeInMs > PUNCH_IN_EXTEND_MECH_WAIT_TIME)
      {
        _currentStep = AQUIRE_HATCH_STEP.EXTEND_MECH_STEP;
      }
    }
    else if(_currentStep == AQUIRE_HATCH_STEP.EXTEND_MECH_STEP)
    {
      _cargo.setMechanism(MECHANISM_POSITION.EXTENDED);
      _currentStep = AQUIRE_HATCH_STEP.WAIT_TO_OPEN;
      _startTimeInMs = System.nanoTime() / 1000000;
    }
    else if(_currentStep == AQUIRE_HATCH_STEP.WAIT_TO_OPEN)
    {
      long currentTimeInMs = System.nanoTime() / 1000000;
      long elapsedTimeInMs = currentTimeInMs - _startTimeInMs;
      if (elapsedTimeInMs > EXTEND_OUT_TO_BEAK_OPEN)
      {
        _currentStep = AQUIRE_HATCH_STEP.BEAK_OPEN_STEP;
      }
    }
    else if(_currentStep == AQUIRE_HATCH_STEP.BEAK_CLOSE_STEP);
    {
      _cargo.setBeak(BEAK_POSITION.OPEN);
      _currentStep = AQUIRE_HATCH_STEP.BEAK_OPEN_STEP;
     
    }
  }

  // Make this return true when this Command no longer needs to run execute()
  @Override
  protected boolean isFinished() {
    return (_currentStep == AQUIRE_HATCH_STEP.BEAK_OPEN_STEP);
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
