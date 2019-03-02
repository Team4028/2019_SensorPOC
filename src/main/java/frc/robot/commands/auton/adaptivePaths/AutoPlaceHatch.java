package frc.robot.commands.auton.adaptivePaths;

import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj.command.CommandGroup;
import edu.wpi.first.wpilibj.command.PrintCommand;
import edu.wpi.first.wpilibj.command.WaitCommand;
import frc.robot.commands.auton.util.printTimeFromStart;
import frc.robot.commands.chassis.DriveSetDistance;
import frc.robot.commands.infeed.ScoreHatch;
import frc.robot.commands.infeed.TogglePunch;
import frc.robot.sensors.GyroNavX;
import frc.robot.sensors.VisionLL;
import frc.robot.subsystems.Chassis;

public class AutoPlaceHatch extends CommandGroup {
    VisionLL _limeLight = VisionLL.getInstance();
    GyroNavX _navX = GyroNavX.getInstance();
    Chassis _chassis = Chassis.getInstance();
    double _startTime;

    public AutoPlaceHatch() 
    {
        setInterruptible(false);
        requires(_chassis);
        addSequential(new Auton_turnFromVision());
        addSequential(new PrintCommand("SECOND VISION TURN TERMINATING"));
        addSequential(new printTimeFromStart());
        addSequential(new DriveVisionDistance(),3);
        addSequential(new PrintCommand("VISION DRIVE STRAIGHT TERMINATING"));
        addSequential(new printTimeFromStart());
        addSequential(new AutonFastPlaceHatch());
        addParallel(new printTimeFromStart());
        addSequential(new DriveSetDistance(-5));
        addSequential(new TogglePunch());
    }

	@Override
    protected void initialize() 
    {
        _startTime = Timer.getFPGATimestamp();
    }

    @Override
    protected boolean isFinished() {
        return super.isFinished() || Timer.getFPGATimestamp()-_startTime>=10;
    }
}