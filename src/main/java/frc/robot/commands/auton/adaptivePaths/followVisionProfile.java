package frc.robot.commands.auton.adaptivePaths;

import edu.wpi.first.wpilibj.command.CommandGroup;
import frc.robot.commands.auton.Auton_RunTimedMotionProfileCommand;
import frc.robot.commands.auton.Auton_ParallelStarter;
import frc.robot.auton.path_planning.problem;

public class followVisionProfile extends CommandGroup{
    
    public followVisionProfile(double timeOut)
    {
        addParallel(new Auton_ParallelStarter());
        // addSequential(new Auton_WaitUntilPathPlanned());
        System.out.println(problem._path);
        addSequential( new Auton_RunTimedMotionProfileCommand(problem._path, timeOut));
    }
}