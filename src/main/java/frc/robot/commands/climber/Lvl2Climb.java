package frc.robot.commands.climber;

import java.util.Arrays;

import edu.wpi.first.wpilibj.command.Command;
import edu.wpi.first.wpilibj.command.CommandGroup;
import edu.wpi.first.wpilibj.command.PrintCommand;
import edu.wpi.first.wpilibj.command.WaitCommand;
import frc.robot.commands.auton.util.Series_Command;
import frc.robot.commands.chassis.DriveWithControllers;
import frc.robot.commands.chassis.StopChassis;
import frc.robot.commands.infeed.SendBeakOut;
import frc.robot.commands.infeed.SendBucketIn;
import frc.robot.commands.infeed.SendBucketOut;
import frc.robot.commands.infeed.TogglePunch;
import frc.robot.subsystems.Climber;

public class Lvl2Climb extends CommandGroup
{
    double climbHeight =-8500;
    double clearedHeight=-696.9;//Nice
    Climber _climber = Climber.getInstance();
    public Lvl2Climb()
    {
        requires(_climber);
        setInterruptible(false);
        //addParallel(new DriveClimber(0.2));
        addSequential(new MoveClimberToPos(climbHeight,0.1));
        addParallel(new DriveClimber(0.4));
        addParallel(new DriveWithControllers(0.2, 0));
        addSequential(new Series_Command(Arrays.asList(new Command[] 
        {
            new PrintCommand("Holding Starts"),
            new HoldClimber(0.6),
            new PrintCommand("Stops Holding"),
            new SendBucketOut(),
            new SendBeakOut(),
            new TogglePunch(),
            new MoveClimberToPos(clearedHeight,0.1),
        })));
        addParallel(new PrintCommand("Moved to Clear Height"));
        addParallel(new DriveWithControllers(0.3, 0),2);
        addSequential(new Series_Command(Arrays.asList(new Command[]
        {
            new WaitCommand(0.75),
            new SendBucketIn()
        })));
        addSequential(new PrintCommand("Driven"));
        addParallel(new StopChassis(),0.25);
        addSequential(new DriveClimber(0.0),0.25);
    }
}