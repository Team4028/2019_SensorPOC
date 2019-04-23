/*----------------------------------------------------------------------------*/
/* Copyright (c) 2018 FIRST. All Rights Reserved.                             */
/* Open Source Software - may be modified and shared by FRC teams. The code   */
/* must be accompanied by the FIRST BSD license file in the root directory of */
/* the project.                                                               */
/*----------------------------------------------------------------------------*/

package frc.robot.commands.auton.autons.Left;

import edu.wpi.first.wpilibj.command.CommandGroup;
import edu.wpi.first.wpilibj.command.WaitCommand;
import frc.robot.commands.auton.DriveOffLevel2Forwards;

public class LSingleHatchLSideLevel2 extends CommandGroup {
  /**
   * Add your docs here.
   */
  public LSingleHatchLSideLevel2() {
    setInterruptible(true);
    addSequential(new DriveOffLevel2Forwards());
    addSequential(new WaitCommand(0.5));
    addSequential(new LSingleHatchLSide());
  }
}
