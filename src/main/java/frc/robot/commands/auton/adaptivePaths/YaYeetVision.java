/*----------------------------------------------------------------------------*/
/* Copyright (c) 2018 FIRST. All Rights Reserved.                             */
/* Open Source Software - may be modified and shared by FRC teams. The code   */
/* must be accompanied by the FIRST BSD license file in the root directory of */
/* the project.                                                               */
/*----------------------------------------------------------------------------*/

package frc.robot.commands.auton.adaptivePaths;

import com.ctre.phoenix.motorcontrol.ControlMode;

import edu.wpi.first.wpilibj.command.Command;
import frc.robot.Constants;
import frc.robot.sensors.DistanceRev2mSensor;
import frc.robot.sensors.VisionLL;
import frc.robot.sensors.VisionLL.LIMELIGHT_PIPELINE;
import frc.robot.subsystems.NEOChassis;
import frc.robot.util.GeneralUtilities;

public class YaYeetVision extends Command 
{
  NEOChassis _chassis = NEOChassis.getInstance();
  VisionLL _limelight = VisionLL.getInstance();
  DistanceRev2mSensor _ds = DistanceRev2mSensor.getInstance();
  double kPDXFIX = 0.005;
  double kPAFIXBig = 0.006;
  double kPAFIXSmall = 0.008;
  double kLowPassFilterCurrentValueWeight = .64;
  double previousTurnCmd = 0.;
  boolean isFirstCycle = false;
  double rawTurnCmd;
  double fwdVBus = 0.36;
  double kAngleOneLimt = 15;
  double kDXLimit = Constants.BIG_NUMBER;
  double kReducedForwardVBus = .21;
  double kReducedForwardVBusDXLimit = 2.5;
  double kReducedForwardVBusAngleOneMinimum = 10;
  double kPReducedForwardVBusBig = .006;
  double kPReducedForwardVBusSmall = .008;

  public enum AUTO_SCORE_STATE
  {
    UNDEFINED,
    A1FIX, 
    REDUCED_FORWARD_VBUS
  }
  AUTO_SCORE_STATE state;
  public YaYeetVision() 
  {
    setInterruptible(true);
    requires(_chassis);
  }

  // Called just before this Command runs the first time
  @Override
  protected void initialize() 
  {
    _limelight.changeLimelightPipeline(LIMELIGHT_PIPELINE.CENTER);
    state = AUTO_SCORE_STATE.UNDEFINED;
    _limelight.turnOnLEDs();
  }

  // Called repeatedly when this Command is scheduled to run
  @Override
  protected void execute() 
  {
    double a1 = _limelight.getTheta();
    System.out.println(" A1: "+GeneralUtilities.roundDouble(a1, 3));



    if (!_limelight.get_isTargetInFOV()){
      state = AUTO_SCORE_STATE.UNDEFINED;
    }
    else if (Math.abs(a1) > kReducedForwardVBusAngleOneMinimum){
      state = AUTO_SCORE_STATE.REDUCED_FORWARD_VBUS;
    }
    else
    {
      state = AUTO_SCORE_STATE.A1FIX;
    }

    double turnCmd=0;

    System.out.println(state);
    if(_limelight.get_isTargetInFOV())
    {
      switch(state)
      {
        case UNDEFINED:
        applyLowPassFilter(0);
        _chassis.stop();
          break;
        case A1FIX:
          //Turn Down A1
          if(Math.abs(a1)>10)
          {
            rawTurnCmd = kPAFIXBig * limit(a1, kAngleOneLimt);
          }
          else
          {
            rawTurnCmd = kPAFIXSmall * limit(a1, kAngleOneLimt);
          }
          turnCmd = applyLowPassFilter(rawTurnCmd);
          _chassis.arcadeDrive(fwdVBus, turnCmd*3.125);//Divide by 0.32
          break;
        case REDUCED_FORWARD_VBUS:
        if(Math.abs(a1)>10)
          {
            rawTurnCmd = kPReducedForwardVBusBig * limit(a1, kAngleOneLimt);
          }
          else
          {
            rawTurnCmd = kPReducedForwardVBusSmall * limit(a1, kAngleOneLimt);
          }
          turnCmd = applyLowPassFilter(rawTurnCmd);
          _chassis.arcadeDrive(kReducedForwardVBus,turnCmd*3.125);
          break;
      }
    }
    else
    {
      _chassis.stop();
    }
  }

  // Make this return true when this Command no longer needs to run execute()
  @Override
  protected boolean isFinished() {
    return !_limelight.get_isTargetInFOV();
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

  public double limit(double value, double cap){
    if (value < -1 * Math.abs(cap)){
      return -1 * Math.abs(cap);
    } else if (value > Math.abs(cap)){
      return Math.abs(cap);
    } else {
      return value;
    }
  }


}
