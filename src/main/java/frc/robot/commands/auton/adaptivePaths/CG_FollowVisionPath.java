package frc.robot.commands.auton.adaptivepaths;

import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj.command.CommandGroup;
import edu.wpi.first.wpilibj.command.PrintCommand;
import frc.robot.commands.auton.util.Auton_ParallelStarter;
import frc.robot.commands.auton.util.printTimeFromStart;
import frc.robot.sensors.GyroNavX;
import frc.robot.sensors.VisionLL;
import frc.robot.sensors.GyroNavX.SCORING_TARGET;
import frc.robot.sensors.GyroNavX.SIDE;
import frc.robot.subsystems.Chassis;

 

public class CG_FollowVisionPath extends CommandGroup {
    VisionLL _limeLight = VisionLL.getInstance();
    GyroNavX _navX = GyroNavX.getInstance();
    Chassis _chassis = Chassis.getInstance();
    double _startTime;

    
    public CG_FollowVisionPath()
    {
        setInterruptible(false);
        requires(_chassis);
        addParallel(new Auton_ParallelStarter());
        addSequential(new Auton_turnFromVision());
        addSequential(new PrintCommand("SECOND VISION TURN TERMINATING"));
        addSequential(new printTimeFromStart());
        addSequential(new DriveVisionDistance(), 1.5);
        addSequential(new PrintCommand("VISION DRIVE STRAIGHT TERMINATING"));
        addSequential(new printTimeFromStart());
    }
    @Override
    protected void initialize() 
    {
        _startTime = Timer.getFPGATimestamp();
    }
    @Override
    protected boolean isFinished() 
    {
        return super.isFinished() || Timer.getFPGATimestamp()-_startTime>=10;
    }
}