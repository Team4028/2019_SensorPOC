package frc.robot.commands.auton.autons.Center;

import edu.wpi.first.wpilibj.command.CommandGroup;
import frc.robot.auton.pathfollowing.Paths;
import frc.robot.auton.pathfollowing.Paths.Right;
import frc.robot.auton.pathfollowing.control.Path;
import frc.robot.commands.auton.RunMotionProfileCommand;
import frc.robot.commands.auton.adaptivePaths.CG_FollowVisionPath;
import frc.robot.commands.chassis.DriveSetDistance;
import frc.robot.commands.chassis.TurnInPlace;

public class CDoubleHatchRFrontRSide extends CommandGroup {

    Path _toFeederStation = Paths.getPath(Right.TO_FEEDER_STATION_FROM_R_FRONT);
    Path _toBay = Paths.getPath(Right.TO_FIRST_BAY_FROM_FEEDER_STATION);
    
    public CDoubleHatchRFrontRSide(){
        setInterruptible(false);
        addSequential(new CSingleHatchRFront());
        addSequential(new DriveSetDistance(-5));
        addSequential(new TurnInPlace(105, true));
        addSequential(new RunMotionProfileCommand(_toFeederStation));
        addSequential(new CG_FollowVisionPath());
        addSequential(new DriveSetDistance(-5));
        addSequential(new TurnInPlace(340, true));
        addSequential(new RunMotionProfileCommand(_toBay));
        addSequential(new CG_FollowVisionPath());
    }
}