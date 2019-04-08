package frc.robot.commands.auton.autons.Left;



import edu.wpi.first.wpilibj.command.CommandGroup;
import edu.wpi.first.wpilibj.command.WaitCommand;
import frc.robot.auton.pathfollowing.control.Path;
import frc.robot.commands.auton.RunMotionProfileCommand;
import frc.robot.commands.auton.StartAcquireHatch;
import frc.robot.commands.auton.adaptivePaths.AutoPlaceHatch;
import frc.robot.commands.auton.adaptivePaths.AutoTrackTarget;
import frc.robot.commands.auton.adaptivePaths.YaYeetVision;
import frc.robot.commands.auton.util.ReverseNavX;
import frc.robot.commands.chassis.DriveSetDistance;
import frc.robot.commands.chassis.TurnInPlace;
import frc.robot.commands.elevator.MoveToPresetPosition;
import frc.robot.commands.infeed.ScoreHatch;
import frc.robot.commands.infeed.SendBucketIn;
import frc.robot.commands.infeed.SendBucketOut;
import frc.robot.commands.infeed.ToggleBeakInOut;
import frc.robot.commands.infeed.ToggleBeakOpenClose;
import frc.robot.commands.infeed.TogglePunch;
import frc.robot.commands.vision.ChoosePipeline;
import frc.robot.sensors.VisionLL.LIMELIGHT_PIPELINE;
import frc.robot.subsystems.Elevator.ELEVATOR_TARGET_POSITION;
import frc.robot.auton.pathfollowing.*;
import frc.robot.auton.pathfollowing.Paths.Left;

public class LSingleHatchBackRocketL extends CommandGroup
{
    Path _toBackRocket = Paths.getPath(Left.BACK_ROCKET);
    
    public LSingleHatchBackRocketL()
    {
        setInterruptible(true);
        addParallel(new ChoosePipeline(LIMELIGHT_PIPELINE.RIGHT_PNP));
        addParallel(new StartAcquireHatch());
        addSequential(new ReverseNavX(true));
        addSequential(new RunMotionProfileCommand(_toBackRocket));
        addSequential(new TurnInPlace(-160, true));
        addSequential(new YaYeetVision());
        // addSequential(new AutoTrackTarget(),4);
        // addSequential(new MoveToPresetPosition(ELEVATOR_TARGET_POSITION.LEVEL_3));
        // addParallel(new SendBucketOut());
        // addSequential(new DriveSetDistance(10),0.5);
        // addSequential(new ToggleBeakOpenClose());
        // addSequential(new TogglePunch());
        // addSequential(new WaitCommand(1));
        // addSequential(new TogglePunch());
        // addSequential(new DriveSetDistance(-5));

    }
}