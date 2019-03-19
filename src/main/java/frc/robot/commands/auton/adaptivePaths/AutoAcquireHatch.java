package frc.robot.commands.auton.adaptivePaths;

import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj.command.CommandGroup;
import edu.wpi.first.wpilibj.command.PrintCommand;
import edu.wpi.first.wpilibj.command.WaitCommand;
import frc.robot.commands.auton.util.Auton_ParallelStarter;
import frc.robot.commands.auton.util.printTimeFromStart;
import frc.robot.commands.chassis.DriveSetDistance;
import frc.robot.commands.infeed.AcquireHatch;
import frc.robot.commands.infeed.ReleaseInfeed;
import frc.robot.commands.infeed.ScoreHatch;
import frc.robot.sensors.GyroNavX;
import frc.robot.sensors.VisionLL;
import frc.robot.sensors.GyroNavX.SCORING_TARGET;
import frc.robot.sensors.GyroNavX.SIDE;
import frc.robot.subsystems.Chassis;

public class AutoAcquireHatch extends CommandGroup {
    VisionLL _limeLight = VisionLL.getInstance();
    GyroNavX _navX = GyroNavX.getInstance();
    Chassis _chassis = Chassis.getInstance();
    double _startTime;

    public AutoAcquireHatch() 
    {
        setInterruptible(false);
        requires(_chassis);
        addSequential(new Auton_turnFromVision());
        addSequential(new PrintCommand("SECOND VISION TURN TERMINATING"));
        addSequential(new printTimeFromStart());
        addSequential(new DriveVisionAcquireDistance(),3);
        addSequential(new WaitCommand(0.25));
        addSequential(new PrintCommand("VISION DRIVE STRAIGHT TERMINATING"));
        addSequential(new printTimeFromStart());
        //addSequential(new ReleaseInfeed());
        addSequential(new AutoFastPickHatch());
        addParallel(new printTimeFromStart());
        addSequential(new DriveSetDistance(-5));
    }

	@Override
    protected void initialize() 
    {
        _startTime = Timer.getFPGATimestamp();
        //_limeLight.setIsInVisionMode(true);
    }

    @Override
    protected boolean isFinished() {
        return super.isFinished() || Timer.getFPGATimestamp()-_startTime>=10;
    }
    @Override
    protected void end() {
        //_limeLight.setIsInVisionMode(false);
    }
}