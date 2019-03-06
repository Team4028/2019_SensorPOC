package frc.robot.commands.climber;

import java.util.Arrays;

import edu.wpi.first.wpilibj.command.Command;
import edu.wpi.first.wpilibj.command.CommandGroup;
import edu.wpi.first.wpilibj.command.WaitCommand;
import edu.wpi.first.wpilibj.command.WaitUntilCommand;
import frc.robot.commands.auton.util.Series_Command;
import frc.robot.commands.auton.util.Simultaneous_Command;
import frc.robot.commands.auton.util.printTimeFromStart;
import frc.robot.commands.chassis.DriveWithControllers;
import frc.robot.commands.chassis.StopChassis;
import frc.robot.commands.infeed.ReleaseInfeed;

public class ClimbSequence extends CommandGroup
{
    double climbHeight =-20500;
    double clearedHeight=-600;

    public ClimbSequence()
    {
        //addParallel(new DriveClimber(0.2));
        addSequential(new MoveClimberToPos(climbHeight));
        addParallel(new DriveClimber(0.4));
        addParallel(new DriveWithControllers(0.2, 0));
        addSequential(new Series_Command(Arrays.asList(new Command[] 
        {
            new HoldClimber(2),
            new ReleaseInfeed(),
            new MoveClimberToPos(clearedHeight),
        })));
        addSequential(new DriveWithControllers(0.3, 0),0.85);
        addSequential(new StopChassis(),0.25);
        addSequential(new DriveClimber(0),0.25);

    }
}