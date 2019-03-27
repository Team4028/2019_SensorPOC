/*----------------------------------------------------------------------------*/
/* Copyright (c) 2018 FIRST. All Rights Reserved.                             */
/* Open Source Software - may be modified and shared by FRC teams. The code   */
/* must be accompanied by the FIRST BSD license file in the root directory of */
/* the project.                                                               */
/*----------------------------------------------------------------------------*/

package frc.robot.commands.auton.adaptivePaths;

import edu.wpi.first.wpilibj.command.CommandGroup;
import edu.wpi.first.wpilibj.command.WaitCommand;
import frc.robot.commands.vision.TurnOnLEDs;
import frc.robot.util.BeakXboxController.Thumbstick;

public class AutoTrackTarget extends CommandGroup {
  /**
   * Add your docs here.
   */
  public AutoTrackTarget() {
    addSequential(new EasierBetterVisionThing());
  }
}
