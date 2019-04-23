package frc.robot.commands.auton.autons.Left;



import edu.wpi.first.wpilibj.command.CommandGroup;
import frc.robot.auton.pathfollowing.control.Path;
import frc.robot.commands.auton.RunMotionProfileCommand;
import frc.robot.commands.auton.StartAcquireHatch;
import frc.robot.commands.auton.adaptivePaths.YaYeetVision;
import frc.robot.commands.auton.util.ReverseNavX;
import frc.robot.commands.chassis.TurnInPlace;
import frc.robot.commands.vision.ChoosePipeline;
import frc.robot.sensors.VisionLL.LIMELIGHT_PIPELINE;
import frc.robot.auton.pathfollowing.*;
import frc.robot.auton.pathfollowing.Paths.Left;

public class LSingleHatchBackRocketL extends CommandGroup
{
    Path _toBackRocket = Paths.getPath(Left.BACK_ROCKET);
    
    public 
    LSingleHatchBackRocketL()
    {
        setInterruptible(true);
        addSequential(new ReverseNavX(true));
        addSequential(new RunMotionProfileCommand(_toBackRocket));
        addSequential(new TurnInPlace(-160, true));


    }
}