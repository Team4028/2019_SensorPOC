/*----------------------------------------------------------------------------*/
/* Copyright (c) 2018 FIRST. All Rights Reserved.                             */
/* Open Source Software - may be modified and shared by FRC teams. The code   */
/* must be accompanied by the FIRST BSD license file in the root directory of */
/* the project.                                                               */
/*----------------------------------------------------------------------------*/

package frc.robot.commands.vision;

import edu.wpi.first.wpilibj.buttons.Button;
import edu.wpi.first.wpilibj.command.Command;
import frc.robot.sensors.GyroNavX;
import frc.robot.sensors.SwitchableCameraServer;
import frc.robot.sensors.VisionLL;
import frc.robot.sensors.VisionLL.LIMELIGHT_PIPELINE;
import frc.robot.subsystems.Cargo;
import frc.robot.util.BeakXboxController.Trigger;

public class ChangeVisionPipeline extends Command {
  VisionLL _vision = VisionLL.getInstance();
  GyroNavX _navX = GyroNavX.getInstance();
  Cargo _cargo = Cargo.getInstance();
  SwitchableCameraServer _camera = SwitchableCameraServer.getInstance();

  Button _dPadUp;      //Limelight Center Target
  Button _dPadUpRight; //Limelight Right Target
  Button _dPadUpLeft;  //Limelight Left Target
  Trigger _rightTrigger;   //Triggers End of Command
  Button _startPipelineBtn;//Starts/Ends tuning

  public ChangeVisionPipeline(Button dPadUp, Button dPadUpRight, 
    Button dPadUpLeft, Trigger rightTrigger, Button startPipelineBtn) {
    requires(_cargo);
    setInterruptible(true);
    _dPadUp = dPadUp;
    _dPadUpRight = dPadUpRight;
    _dPadUpLeft = dPadUpLeft;
    _rightTrigger = rightTrigger;
    _startPipelineBtn = startPipelineBtn;
  }

  // Called just before this Command runs the first time
  @Override
  protected void initialize() {
  }

  // Called repeatedly when this Command is scheduled to run
  @Override
  protected void execute() {
    _camera.displayLimelight();
    if(_dPadUp.get()) {
      _vision.changeLimelightPipeline(LIMELIGHT_PIPELINE.CENTER);
    }
    else if(_dPadUpRight.get()){
      _vision.changeLimelightPipeline(LIMELIGHT_PIPELINE.RIGHT);
    }
    else if(_dPadUpLeft.get()){
      _vision.changeLimelightPipeline(LIMELIGHT_PIPELINE.LEFT);
    }
  }

  // Make this return true when this Command no longer needs to run execute()
  @Override
  protected boolean isFinished() {
    return _rightTrigger.get() || !_startPipelineBtn.get();
  }

  // Called once after isFinished returns true
  @Override
  protected void end() {}

  // Called when another command which requires one or more of the same
  // subsystems is scheduled to run
  @Override
  protected void interrupted() {_vision.changeLimelightPipeline(LIMELIGHT_PIPELINE.CENTER);}
}
