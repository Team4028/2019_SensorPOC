/*----------------------------------------------------------------------------*/
/* Copyright (c) 2018 FIRST. All Rights Reserved.                             */
/* Open Source Software - may be modified and shared by FRC teams. The code   */
/* must be accompanied by the FIRST BSD license file in the root directory of */
/* the project.                                                               */
/*----------------------------------------------------------------------------*/

package frc.robot.commands.auton.autons;

import edu.wpi.first.wpilibj.command.CommandGroup;
import frc.robot.commands.auton.DriveOffLevel2Forwards;

public class Level2TeleopSandstorm extends CommandGroup {
  /**
   * Add your docs here.
   */
  public Level2TeleopSandstorm() {
    setInterruptible(true);
    addSequential(new DriveOffLevel2Forwards());
  }
}
