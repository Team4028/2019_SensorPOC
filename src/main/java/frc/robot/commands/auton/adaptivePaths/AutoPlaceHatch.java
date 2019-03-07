package frc.robot.commands.auton.adaptivePaths;

import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj.buttons.Button;
import edu.wpi.first.wpilibj.command.CommandGroup;
import edu.wpi.first.wpilibj.command.PrintCommand;
import edu.wpi.first.wpilibj.command.WaitCommand;
import frc.robot.commands.auton.util.printTimeFromStart;
import frc.robot.commands.chassis.DriveSetDistance;
import frc.robot.commands.chassis.StopChassis;
import frc.robot.commands.elevator.MoveToPresetPosition;
import frc.robot.commands.infeed.ReleaseInfeed;
import frc.robot.commands.infeed.ScoreHatch;
import frc.robot.commands.infeed.SendBucketOut;
import frc.robot.commands.infeed.TogglePunch;
import frc.robot.commands.teleop.MovetoStoredPosition;
import frc.robot.sensors.GyroNavX;
import frc.robot.sensors.VisionLL;
import frc.robot.subsystems.Cargo;
import frc.robot.subsystems.Chassis;
import frc.robot.subsystems.Elevator.ELEVATOR_TARGET_POSITION;
import frc.robot.util.BeakXboxController.Trigger;

public class AutoPlaceHatch extends CommandGroup {
    VisionLL _limeLight = VisionLL.getInstance();
    GyroNavX _navX = GyroNavX.getInstance();
    Chassis _chassis = Chassis.getInstance();
    double _startTime;
    Cargo _cargo = Cargo.getInstance();
    Trigger _rt;
    boolean _isAuton;

    public AutoPlaceHatch() 
    {
        _isAuton=true;
        setInterruptible(false);
        addParallel(new SendBucketOut());
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

    public AutoPlaceHatch(Trigger rt)
    {
        _isAuton=false;
        _rt=rt;
        setInterruptible(false);
        addParallel(new SendBucketOut());
        addSequential(new Auton_turnFromVision());
        addSequential(new MovetoStoredPosition(),2);
        addSequential(new PrintCommand("SECOND VISION TURN TERMINATING"));
        addSequential(new printTimeFromStart());
        addSequential(new DriveVisionDistance(),3);
        addParallel(new PrintCommand("VISION DRIVE STRAIGHT TERMINATING"));
        addSequential(new printTimeFromStart());
        /*addSequential(new AutonFastPlaceHatch());
        addParallel(new printTimeFromStart());
        addSequential(new DriveSetDistance(-15));
        addSequential(new MoveToPresetPosition(ELEVATOR_TARGET_POSITION.HOME));
        addParallel(new ReleaseInfeed());
        addSequential(new TogglePunch());*/
    }

	@Override
    protected void initialize() 
    {
        _startTime = Timer.getFPGATimestamp();
    }

    @Override
    protected boolean isFinished() {   
        if(!_isAuton)  
        {
            if(!_rt.get())
            {
                return true;
            }
        }   
        return super.isFinished() || Timer.getFPGATimestamp()-_startTime>=10;
    }
    @Override
    protected void end() {
        _chassis.stop();
    }
    @Override
    protected void interrupted() {
        _chassis.stop();
    }
    
}