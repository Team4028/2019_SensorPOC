/*----------------------------------------------------------------------------*/
/* Copyright (c) 2018 FIRST. All Rights Reserved.                             */
/* Open Source Software - may be modified and shared by FRC teams. The code   */
/* must be accompanied by the FIRST BSD license file in the root directory of */
/* the project.                                                               */
/*----------------------------------------------------------------------------*/

package frc.robot.commands.auton.adaptivePaths;



import com.ctre.phoenix.motorcontrol.ControlMode;

import edu.wpi.first.wpilibj.command.Command;
import frc.robot.sensors.DistanceRev2mSensor;
import frc.robot.sensors.VisionLL;
import frc.robot.subsystems.Chassis;
import frc.robot.util.GeneralUtilities;

public class LastGaspVision extends Command 
{
  Chassis _chassis = Chassis.getInstance();
  VisionLL _limelight = VisionLL.getInstance();
  DistanceRev2mSensor _ds = DistanceRev2mSensor.getInstance();
  public double kPDXFIX = 0.005;
  public double kPAFIX = 0.004;
  public double kLowPassFilterCurrentValueWeight = .5;
  public double previousTurnCmd = 0.;
  public boolean isFirstCycle = false;
  public double rawTurnCmd;
  public double fwdVBus=0.25;

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
    _limelight.turnOnLEDs();
  }

  // Called repeatedly when this Command is scheduled to run
  @Override
  protected void execute() 
  {
    double dx = _limelight.get_xOffset()+6;
    System.out.print("DX: "+GeneralUtilities.roundDouble(dx, 3));
    double a1 = _limelight.getTheta()-3.25;
    System.out.println(" A1: "+GeneralUtilities.roundDouble(a1, 3));


    if(dx==7)
    {
      state=AUTO_SCORE_STATE.UNDEFINED;
    }
    else if(Math.abs(dx) > 15)
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

    double turnCmd=0;


      switch(state)
      {
        case UNDEFINED:
        applyLowPassFilter(0);
        _chassis.stop();
          break;
        
        case DXFIX:
          /*
          Do something to make the turn command reduce dx
          have a a1 cap
          if a1 exceeds the cap, turn the a1 down 
          */
          if(Math.abs(a1)<20)
          {
            rawTurnCmd = -kPDXFIX*dx;
            turnCmd = applyLowPassFilter(rawTurnCmd);
            _chassis.setLeftRightCommand(ControlMode.PercentOutput,fwdVBus+turnCmd,fwdVBus-turnCmd);
          }
          else
          {
            rawTurnCmd = kPAFIX*a1;
            turnCmd = applyLowPassFilter(rawTurnCmd);
            _chassis.setLeftRightCommand(ControlMode.PercentOutput,fwdVBus+turnCmd,fwdVBus-turnCmd);
          }
          break;
        case A1FIX:
          //Turn Down A1
          rawTurnCmd = kPAFIX*a1;
          turnCmd = applyLowPassFilter(rawTurnCmd);
          _chassis.setLeftRightCommand(ControlMode.PercentOutput,fwdVBus+turnCmd, fwdVBus-turnCmd);
          break;
      }
  }

  // Make this return true when this Command no longer needs to run execute()
  @Override
  protected boolean isFinished() {
    return ((_ds.get_distanceToTargetInInches()>25) && (_ds.get_distanceToTargetInInches()>0))|| (!_limelight.get_isPingable())|| !_limelight.get_isTargetInFOV();
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
    _limelight.turnOffLEDs();
    _chassis.stop();
  }

  public double applyLowPassFilter(double newTurnCmd){
    double newerTurnCmd;
    if (! isFirstCycle){
      newerTurnCmd = kLowPassFilterCurrentValueWeight * newTurnCmd + (1 - kLowPassFilterCurrentValueWeight) * previousTurnCmd;
      previousTurnCmd = newerTurnCmd;
      return newerTurnCmd;
    } else {
      isFirstCycle = true;
      previousTurnCmd = newTurnCmd;
      newerTurnCmd = newTurnCmd;
    }
    return newerTurnCmd;
  }


}
