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
import frc.robot.sensors.DistanceRev2mSensor;
import frc.robot.sensors.VisionLL;
import frc.robot.sensors.VisionLL.LIMELIGHT_PIPELINE;
import frc.robot.subsystems.Chassis;
import frc.robot.util.GeneralUtilities;
import frc.robot.util.BeakXboxController.Thumbstick;

public class EasierBetterVisionThing extends Command {
  Chassis _chassis = Chassis.getInstance();
  Thumbstick _rightThumbstick,_leftThumbstick;
  double kP, kI, kD;
  VisionLL _limelight = VisionLL.getInstance();
  DistanceRev2mSensor _ds = DistanceRev2mSensor.getInstance();
  boolean isFirstCycle;
  double prevError;
  double prevD;
  double prevTime;
  double P,I,D;
  double pushoutDistance;

  public EasierBetterVisionThing(Thumbstick leftThumbstick, Thumbstick rightThumbstick) 
  {
    requires(_chassis);
    setInterruptible(false);
    _leftThumbstick = leftThumbstick;
    _rightThumbstick = rightThumbstick;
  }

  // Called just before this Command runs the first time
  @Override
  protected void initialize() {
    _limelight.changeLimelightPipeline(LIMELIGHT_PIPELINE.CENTER_PNP);
    kP=0.025;
    kI=0;
    kD=0.00;
    P=0;
    I=0;
    D=0;
    prevError=0;
    prevD=0;
    pushoutDistance = 30;
    prevTime = Timer.getFPGATimestamp();
  }

  // Called repeatedly when this Command is scheduled to run
  @Override
  protected void execute() {
    double a2 = 180/Math.PI*Math.atan2(_limelight.get_xOffset()+7, _limelight.get_yOffset());
    double a2Rad = (Math.PI/180) * a2;
    double rawError = _limelight.getTheta();
    double dist = Math.hypot(_limelight.get_xOffset() + 7, _limelight.get_yOffset());
    double littleAngle = (180/Math.PI)*(Math.PI/2 - Math.atan2(((dist - pushoutDistance*Math.cos(a2Rad))/Math.sin(a2Rad)), pushoutDistance));
    double error = rawError - littleAngle;
    System.out.println("Lit Angle: " + littleAngle+180);
    System.out.println("Old Error: " + rawError);
    System.out.println("New Error: " + error+180);
    if(a2<-90)
    {
      a2+=180;
    }
    if(a2>90)
    {
      a2-=180;
    }
    if(Math.abs(error)>5)
    {
        error=Math.copySign(5, error);
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
    double turnCmd = P+I+D;
    if(Math.abs(turnCmd)>0.1)
    {
      turnCmd=Math.copySign(0.1, turnCmd);
    }
    if(_rightThumbstick.get())
    {
      // _chassis.arcadeDrive(0.5*_leftThumbstick.getY(), _rightThumbstick.getX());
    }
    else
    {
      if(Math.abs(a2)<10)
      {
        if(_limelight.get_isTargetInFOV())
        {
          if(error*(7+_limelight.get_xOffset())>0)
          {
            // _chassis.arcadeDrive(0.5*_leftThumbstick.getY(), turnCmd);
            System.out.print(" E: " + GeneralUtilities.roundDouble(error, 3));
            System.out.print(" P: "+GeneralUtilities.roundDouble(P, 3));
            System.out.print(" I: "+GeneralUtilities.roundDouble(I, 3));
            System.out.print(" D: "+GeneralUtilities.roundDouble(D, 3));
            System.out.println("Auto Turn Cmd: "+ turnCmd);    
          }
          else
          {
            System.out.println("Fixing the Garbage Drivers Tendancy to be Garbage");
            //_chassis.stop();
            //_chassis.setLeftRightCommand(ControlMode.PercentOutput, -Math.copySign(0.25, _limelight.get_xOffset()), Math.copySign(0.25, _limelight.get_xOffset()));
          }
        }
        else
        {
          _chassis.stop();
          System.out.println("LL Lost View of Target");
        }
      }
      else
      {
        _chassis.stop();
        System.out.println("Too Offset for Valid Vision: "+ a2);
      }
    }
  }

  // Make this return true when this Command no longer needs to run execute()
  @Override
  protected boolean isFinished() {
    return _ds.get_distanceToTargetInInches()<25 && _ds.get_distanceToTargetInInches()>0;
  }

  // Called once after isFinished returns true
  @Override
  protected void end() {
    System.out.println("Finished");
    _chassis.stop();
  }

  // Called when another command which requires one or more of the same
  // subsystems is scheduled to run
  @Override
  protected void interrupted() {
  }
}
