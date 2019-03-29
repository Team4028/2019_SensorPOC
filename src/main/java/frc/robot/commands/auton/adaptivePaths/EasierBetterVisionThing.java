/*----------------------------------------------------------------------------*/
/* Copyright (c) 2018 FIRST. All Rights Reserved.                             */
/* Open Source Software - may be modified and shared by FRC teams. The code   */
/* must be accompanied by the FIRST BSD license file in the root directory of */
/* the project.                                                               */
/*----------------------------------------------------------------------------*/

package frc.robot.commands.auton.adaptivePaths;

import com.ctre.phoenix.motorcontrol.ControlMode;

import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj.command.Command;
import frc.robot.auton.pathfollowing.util.beakCircularBuffer;
import frc.robot.sensors.DistanceRev2mSensor;
import frc.robot.sensors.VisionLL;
import frc.robot.sensors.VisionLL.LIMELIGHT_PIPELINE;
import frc.robot.subsystems.Chassis;
import frc.robot.util.GeneralUtilities;
import frc.robot.util.BeakXboxController.Thumbstick;

public class EasierBetterVisionThing extends Command {
  Chassis _chassis = Chassis.getInstance();
  double kPAngleOneSmall, kIAngleOneSmall, kDAngleOneSmall;
  VisionLL _limelight = VisionLL.getInstance();
  DistanceRev2mSensor _ds = DistanceRev2mSensor.getInstance();
  boolean isFirstCycle;
  double prevError;
  double prevD;
  double prevTime;
  double P,I,D;
  double pushoutDistance;
  double speedAdjustment;
  beakCircularBuffer angleOneBuffer = new beakCircularBuffer(10);
  double kFindTargetTurnVBus;
  boolean isA2Small;
  boolean forcedFinish;
  double kPAngleOneLarge;
  double kIAngleOneLarge;
  double kDAngleOneLarge;
  double kAngleOneSmallDeadband = 10;
  double kP, kI, kD;
  double kAngleOneSmallTurnVBUSLimit;
  double kAngleOneLargeTurnVBUSLimit;
  double kForwardVBus;
  boolean isInPlaceTurn;
  boolean hasTurnedInPlace;
  double inPlaceTurnKp;
  double inPlaceTurnKd;
  double kInPlaceTurnDeadband;
  boolean hasInPlacedTurned;
  double kMinInPlaceVBUS = 0;
  double currentTime;
  double prevTurnCmd;

  public EasierBetterVisionThing() 
  {
    requires(_chassis);
    setInterruptible(true);

  }

  // Called just before this Command runs the first time
  @Override
  protected void initialize() {
    _limelight.turnOnLEDs();
    _chassis.setRampRate(0.1);
    _limelight.changeLimelightPipeline(LIMELIGHT_PIPELINE.CENTER_PNP);
    P=0;
    I=0;
    D=0;
    prevError=0;
    prevD=0;
    pushoutDistance = 50;
    prevTime = Timer.getFPGATimestamp();
    speedAdjustment=2;
    kFindTargetTurnVBus = .3;
    isA2Small=false;
    kPAngleOneSmall=0.002;
    kIAngleOneSmall=0;
    kDAngleOneSmall=0.09;
    kPAngleOneLarge = .008;
    kIAngleOneLarge = 0;
    kDAngleOneLarge = .02;
    kP = kPAngleOneSmall;
    kD = kDAngleOneSmall;
    kAngleOneLargeTurnVBUSLimit = .4;
    kAngleOneSmallTurnVBUSLimit = .25;
    kForwardVBus = .35;
    isInPlaceTurn = false;
    hasInPlacedTurned = false;
    inPlaceTurnKp = .03;
    inPlaceTurnKd = .105;
    kInPlaceTurnDeadband = 6;
    prevTurnCmd=0;
  }

  // Called repeatedly when this Command is scheduled to run
  @Override
  protected void execute() {
    double dx = _limelight.get_xOffset()+7;
    if(dx<0)
    {
      pushoutDistance=40;
    }
    else
    {
      pushoutDistance = 50;
    }
    double a2 = 180/Math.PI*Math.atan2(dx, _limelight.get_yOffset()-20);
    if(a2>90)
    {
      a2=180-a2;
    }
    if(a2<-90)
    {
      a2=-180-a2;
    }
    double error;
    if(Math.abs(dx)>3 && dx!=7 && !isA2Small)
    {
      isA2Small=false;

      double a2Rad = (Math.PI/180) * a2;
      double rawError = _limelight.getTheta();
      double dist = Math.hypot(_limelight.get_xOffset() + 7, _limelight.get_yOffset()-20);
      double littleAngle = Math.abs((180/Math.PI)*(Math.PI/2 - Math.atan2(((dist - pushoutDistance*Math.cos(a2Rad))/Math.sin(a2Rad)), pushoutDistance)));
      if(littleAngle>90)
      {
        littleAngle = 180-littleAngle;
      }
      error = rawError - Math.copySign(littleAngle, _limelight.get_xOffset()+7);
      //System.out.println("Lit Angle: " + littleAngle);
      //System.out.println("Old Error: " + rawError);
      System.out.println("New Error: " + error);
      //System.out.println(a2);
    }
    else
    {      
      if(dx!=7)
      {
        isA2Small=true;
        if (!hasInPlacedTurned){
          isInPlaceTurn = true;
          hasInPlacedTurned = false;
        } else {
          isInPlaceTurn = false;
        }

      }
      error = _limelight.getTheta();
      System.out.println("SWITCHING"+ error);
    }

    if(Math.abs(_limelight.getTheta()) < kAngleOneSmallDeadband){
      kP = kPAngleOneSmall;
      kI = kIAngleOneSmall;
      kD = kDAngleOneSmall;
    } else {
      kP = kPAngleOneLarge;
      kI = kIAngleOneLarge;
      kD = kDAngleOneLarge;
    }
    if (isInPlaceTurn){
      kP = inPlaceTurnKp;
      kI = 0;
      kD = inPlaceTurnKd;
    }

    error+=3;
    if(Math.abs(error)>15)
    {
        error=Math.copySign(15, error);
        I=0;
    }
    else
    {
        I += kI * error * (Timer.getFPGATimestamp()-prevTime);
    }
    P = kP * error;

        
    if(isFirstCycle)
    {
        D=0;
        isFirstCycle=false;
    }
    else
    {
        D = (kD*(error-prevError)/(Timer.getFPGATimestamp()-prevTime))*0.3+0.7*prevD;           
    }
    if(D>1)
    {
      D=0;
    }
    double turnCmd = P+I+D;
    if (Math.abs(_limelight.getTheta()) < kAngleOneSmallDeadband){
      if(Math.abs(turnCmd)>kAngleOneSmallTurnVBUSLimit)
      {
        turnCmd = Math.copySign(kAngleOneSmallTurnVBUSLimit, turnCmd);
      }
    } else {
      if(Math.abs(turnCmd) > kAngleOneLargeTurnVBUSLimit)
      {
        turnCmd = Math.copySign(kAngleOneLargeTurnVBUSLimit, turnCmd);
      }
    }
        if(_limelight.get_isTargetInFOV())
        {
          angleOneBuffer.addLast(_limelight.getTheta());
          if (isInPlaceTurn){
            _chassis.arcadeDrive(0, turnCmd + Math.copySign(kMinInPlaceVBUS, turnCmd));
            if (Math.abs(error) < kInPlaceTurnDeadband){
              isInPlaceTurn = false;
              hasInPlacedTurned = true;
              System.out.println("IN PLACE TURN TERMINATING");
              _chassis.stop();
              currentTime = Timer.getFPGATimestamp();
            }
          } else {
            if(Timer.getFPGATimestamp()-currentTime>0.5)
            {
              _chassis.arcadeDrive(kForwardVBus, turnCmd);
              prevTurnCmd=turnCmd;
            }
            else
            {
              _chassis.stop();
            }
          }

          //System.out.print(" E: " + GeneralUtilities.roundDouble(error, 3));
          System.out.print(" P: "+GeneralUtilities.roundDouble(P, 3));
          //System.out.print(" I: "+GeneralUtilities.roundDouble(I, 3));
          System.out.print(" D: "+GeneralUtilities.roundDouble(D, 3));
          //System.out.println("Auto Turn Cmd: "+ turnCmd);    
        }
        else
        {
          if(isA2Small)
          {
            _chassis.arcadeDrive(kForwardVBus, 0);
            // double avgAngleOne=0;
            // for (int i = 0; i < 10; i++)
            // {
            //   avgAngleOne += .1 * angleOneBuffer.toArray()[i];
            // }
            // if(avgAngleOne<10)
            // {
            //   _chassis.arcadeDrive(_leftThumbstick.getY(), 0);
            //   System.out.println("Going Straight Lost LL");
            // }
          }
          else
          {
            _chassis.stop();
          }
        }
        // else
        // {
        //   double avgAngleOne = 0;
        //   for (int i = 0; i < 10; i++){
        //     avgAngleOne += .1 * angleOneBuffer.toArray()[i];
        //   }
        //   double turnSgn = Math.signum(avgAngleOne);
        //   // _chassis.stop();
        //   _chassis.arcadeDrive(0., turnSgn * kFindTargetTurnVBus);
        //   System.out.println("LL Lost View of Target. Turn Bus: " + turnSgn * kFindTargetTurnVBus);
        // }
      
    
  }

  // Make this return true when this Command no longer needs to run execute()
  @Override
  protected boolean isFinished() {
    return (forcedFinish || _ds.get_distanceToTargetInInches()<25 && _ds.get_distanceToTargetInInches()>0) && hasInPlacedTurned;
  }

  // Called once after isFinished returns true
  @Override
  protected void end() {
    System.out.println("Terminating");
    _chassis.stop();
    _chassis.setRampRate(0.7);
  }

  // Called when another command which requires one or more of the same
  // subsystems is scheduled to run
  @Override
  protected void interrupted() {
    _limelight.turnOffLEDs();
    _chassis.setRampRate(0.7);
  }

}
