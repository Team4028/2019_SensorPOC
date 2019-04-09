/*----------------------------------------------------------------------------*/
/* Copyright (c) 2018 FIRST. All Rights Reserved.                             */
/* Open Source Software - may be modified and shared by FRC teams. The code   */
/* must be accompanied by the FIRST BSD license file in the root directory of */
/* the project.                                                               */
/*----------------------------------------------------------------------------*/

package frc.robot.commands.infeed;

import edu.wpi.first.wpilibj.command.Command;
import edu.wpi.first.wpilibj.command.CommandGroup;
import frc.robot.commands.infeed.ToggleBeakOpenClose;
import frc.robot.subsystems.Cargo;

public class AutoFastPickHatch extends CommandGroup {
  Cargo _cargo = Cargo.getInstance();
  public AutoFastPickHatch() {
    setInterruptible(false);
    addSequential(new ToggleBeakOpenClose());
  }

  
  @Override
  protected boolean isFinished() {
    return _cargo.get_isBeakOpen();
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
