/*----------------------------------------------------------------------------*/
/* Copyright (c) 2018 FIRST. All Rights Reserved.                             */
/* Open Source Software - may be modified and shared by FRC teams. The code   */
/* must be accompanied by the FIRST BSD license file in the root directory of */
/* the project.                                                               */
/*----------------------------------------------------------------------------*/

package frc.robot.commands.auton;

import com.ctre.phoenix.motorcontrol.ControlMode;

import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj.command.Command;
import frc.robot.subsystems.NEOChassis;

public class DriveOffLevelTwoBackwards extends Command {
  NEOChassis _chassis = NEOChassis.getInstance();
  double _startTime;
  public DriveOffLevelTwoBackwards() {
    requires(_chassis);

  }

  // Called just before this Command runs the first time
  @Override
  protected void initialize() {
    _startTime = Timer.getFPGATimestamp();
  }

  // Called repeatedly when this Command is scheduled to run
  @Override
  protected void execute() {
    _chassis.arcadeDrive(-0.5, 0);
  }

  // Make this return true when this Command no longer needs to run execute()
  @Override
  protected boolean isFinished() {
    return Timer.getFPGATimestamp()-_startTime>1;
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
  }
}
