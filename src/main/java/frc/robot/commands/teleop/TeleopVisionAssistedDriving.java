/*----------------------------------------------------------------------------*/
/* Copyright (c) 2018 FIRST. All Rights Reserved.                             */
/* Open Source Software - may be modified and shared by FRC teams. The code   */
/* must be accompanied by the FIRST BSD license file in the root directory of */
/* the project.                                                               */
/*----------------------------------------------------------------------------*/

package frc.robot.commands.teleop;

import edu.wpi.first.wpilibj.buttons.Button;
import edu.wpi.first.wpilibj.command.CommandGroup;
import frc.robot.commands.elevator.MoveToPresetPosition;
import frc.robot.commands.vision.ChangeVisionPipeline;
import frc.robot.util.BeakXboxController.Trigger;

public class TeleopVisionAssistedDriving extends CommandGroup {
  /**
   * Add your docs here.
   */
  public TeleopVisionAssistedDriving(Button dPadUp, Button dPadUpRight, 
  Button dPadUpLeft, Trigger rightTrigger) {
    addParallel(new ChangeVisionPipeline(dPadUp, dPadUpRight, dPadUpLeft, rightTrigger));
    //Auton Adds Mumbo Jumbo here
    //addSequential(new MoveToPresetPosition());
  }
}
