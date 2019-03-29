package frc.robot.commands.auton.autons.Left;

import edu.wpi.first.wpilibj.command.CommandGroup;
import frc.robot.auton.pathfollowing.Paths;
import frc.robot.auton.pathfollowing.Paths.Left;
import frc.robot.auton.pathfollowing.control.Path;
import frc.robot.commands.auton.RunMotionProfileCommand;
import frc.robot.commands.auton.adaptivePaths.AutoAcquireHatch;
import frc.robot.commands.auton.adaptivePaths.AutoTrackTarget;
import frc.robot.commands.auton.util.Auton_ParallelStarter;
import frc.robot.commands.auton.util.printTimeFromStart;
import frc.robot.commands.elevator.MoveToPresetPosition;
import frc.robot.commands.infeed.AcquireHatch;
import frc.robot.commands.infeed.SendBucketOut;
import frc.robot.subsystems.Chassis;
import frc.robot.subsystems.Elevator.ELEVATOR_TARGET_POSITION;

public class LDoubleHatchRocketL extends CommandGroup
{
    Chassis _chassis = Chassis.getInstance();
    Path _awayFromRocket = Paths.getPath(Left.AWAY_FROM_BACK_ROCKET);
    Path _toFeederStation = Paths.getPath(Left.TO_FEEDER_STATION_FROM_ROCKET);
    public LDoubleHatchRocketL()
    {
        setInterruptible(true);
        addSequential(new LSingleHatchBackRocketL());
        addParallel(new MoveToPresetPosition(ELEVATOR_TARGET_POSITION.HOME));
        addSequential(new RunMotionProfileCommand(_awayFromRocket));
        addSequential(new RunMotionProfileCommand(_toFeederStation));
        addSequential(new AutoTrackTarget());
        addSequential(new AcquireHatch());
        addSequential(new printTimeFromStart());
    }
    @Override
    protected boolean isFinished() {
        return super.isFinished()|| _chassis.getForcedAutonFinish();
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