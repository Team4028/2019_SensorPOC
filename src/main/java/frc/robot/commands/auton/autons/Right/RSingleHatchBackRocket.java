package frc.robot.commands.auton.autons.Right;



import edu.wpi.first.wpilibj.command.CommandGroup;
import edu.wpi.first.wpilibj.command.WaitCommand;
import frc.robot.auton.pathfollowing.control.Path;
import frc.robot.commands.auton.RunMotionProfileCommand;
import frc.robot.commands.auton.StartAcquireHatch;
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
import frc.robot.auton.pathfollowing.Paths.Right;

public class RSingleHatchBackRocket extends CommandGroup
{
    Path _toBackRocket = Paths.getPath(Right.TO_BACK_ROCKET_R);
    
    public RSingleHatchBackRocket()
    {
        setInterruptible(true);
        addSequential(new ReverseNavX(true));
        addSequential(new RunMotionProfileCommand(_toBackRocket));
        addSequential(new TurnInPlace(160, false));
        

    }
}