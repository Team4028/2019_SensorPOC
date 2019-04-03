/*----------------------------------------------------------------------------*/
/* Copyright (c) 2018 FIRST. All Rights Reserved.                             */
/* Open Source Software - may be modified and shared by FRC teams. The code   */
/* must be accompanied by the FIRST BSD license file in the root directory of */
/* the project.                                                               */
/*----------------------------------------------------------------------------*/

package frc.robot.commands.auton.adaptivePaths;

import javax.lang.model.util.ElementScanner6;

import edu.wpi.first.wpilibj.command.Command;
import frc.robot.sensors.DistanceRev2mSensor;
import frc.robot.sensors.VisionLL;
import frc.robot.subsystems.Chassis;

public class LastGaspVision extends Command 
{
  Chassis _chassis = Chassis.getInstance();
  VisionLL _limelight = VisionLL.getInstance();
  DistanceRev2mSensor _ds = DistanceRev2mSensor.getInstance();
  public double kPDXFIX = 0;
  public double kPAFIX = 0;

  public enum AUTO_SCORE_STATE
  {
    UNDEFINED,
    DXFIX,
    A1FIX
  }
  AUTO_SCORE_STATE state;
  public LastGaspVision() 
  {
    setInterruptible(true);
    requires(_chassis);
  }

  // Called just before this Command runs the first time
  @Override
  protected void initialize() 
  {
    state = AUTO_SCORE_STATE.UNDEFINED;
  }

  // Called repeatedly when this Command is scheduled to run
  @Override
  protected void execute() 
  {
    double dx = _limelight.get_xOffset()+7;
    double a1 = _limelight.getTheta();
    if(Math.abs(dx) > 5)
    {
      state = AUTO_SCORE_STATE.DXFIX;
    }
    else
    {
      state = AUTO_SCORE_STATE.A1FIX;
    }

    if (!_limelight.get_isTargetInFOV()){
      state = AUTO_SCORE_STATE.UNDEFINED;
    }

    double turnCmd;

    switch(state)
    {
      case UNDEFINED:
        break;
      
      case DXFIX:
        /*
        Do something to make the turn command reduce dx
        have a a1 cap
        if a1 exceeds the cap, turn the a1 down 
        */
        if(Math.abs(a1)<10)
        {
          turnCmd = kPDXFIX*dx;
        }
        else
        {
          turnCmd = kPAFIX*a1;
        }
        _chassis.arcadeDrive(0.2, turnCmd);
      case A1FIX:
        //Turn Down A1
        turnCmd = kPAFIX*a1;
        _chassis.arcadeDrive(0.2, turnCmd);
        break;
    }
  }

  // Make this return true when this Command no longer needs to run execute()
  @Override
  protected boolean isFinished() {
    return ((_ds.get_distanceToTargetInInches()>25) && (_ds.get_distanceToTargetInInches()>0))|| (!_limelight.get_isPingable());
  }

  // Called once after isFinished returns true
  @Override
  protected void end() {
    _chassis.stop();
  }

  // Called when another command which requires one or more of the same
  // subsystems is scheduled to run
  @Override
  protected void interrupted() {
    _chassis.stop();
  }


}
