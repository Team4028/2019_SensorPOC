package frc.robot.commands.auton.autons.Left;

import edu.wpi.first.wpilibj.command.CommandGroup;
import edu.wpi.first.wpilibj.command.WaitCommand;
import frc.robot.auton.pathfollowing.Paths;
import frc.robot.auton.pathfollowing.Paths.Left;
import frc.robot.auton.pathfollowing.control.Path;
import frc.robot.commands.auton.RunMotionProfileCommand;
import frc.robot.commands.auton.StartAcquireHatch;
import frc.robot.commands.auton.adaptivePaths.AutoPlaceHatch;
import frc.robot.commands.auton.adaptivePaths.AutoTrackTarget;
import frc.robot.commands.auton.util.printTimeFromStart;
import frc.robot.commands.chassis.DriveSetDistance;
import frc.robot.commands.chassis.TurnInPlace;
import frc.robot.commands.infeed.SendBucketOut;
import frc.robot.commands.infeed.ToggleBeakOpenClose;
import frc.robot.commands.infeed.TogglePunch;
import frc.robot.commands.vision.ChoosePipeline;
import frc.robot.sensors.VisionLL.LIMELIGHT_PIPELINE;
;

public class LSingleHatchLSide extends CommandGroup {
    Path _sidePath = Paths.getPath(Left.TO_LEFT_CARGO_SHIP_FIRST);
    public LSingleHatchLSide() {
        setInterruptible(false);
        addSequential(new ChoosePipeline(LIMELIGHT_PIPELINE.RIGHT_PNP));
        addSequential(new StartAcquireHatch());
        addSequential(new RunMotionProfileCommand(_sidePath));
        addSequential(new TurnInPlace(90, true));
        addParallel(new SendBucketOut());
        addParallel(new printTimeFromStart());
        addSequential(new WaitCommand(0.2));
        addSequential(new AutoTrackTarget(),4);
        addSequential(new ToggleBeakOpenClose());
        addSequential(new TogglePunch());
        addSequential(new WaitCommand(0.5));
        addSequential(new DriveSetDistance(-5));
    }
}