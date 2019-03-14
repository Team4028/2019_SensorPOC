/*----------------------------------------------------------------------------*/
/* Copyright (c) 2018 FIRST. All Rights Reserved.                             */
/* Open Source Software - may be modified and shared by FRC teams. The code   */
/* must be accompanied by the FIRST BSD license file in the root directory of */
/* the project.                                                               */
/*----------------------------------------------------------------------------*/

package frc.robot.commands.infeed;

import edu.wpi.first.wpilibj.command.Command;
import frc.robot.subsystems.Cargo;

public class SendBeakOut extends Command {
  Cargo _cargo = Cargo.getInstance();
  public SendBeakOut()
  {
      setInterruptible(false);
  }
  @Override
  protected void initialize() {
      if(!_cargo.get_isBeakOut())
      {
          _cargo.toggleBeakInOut();
      }
  }

  @Override
  protected boolean isFinished() {
      return true;
  }
}
