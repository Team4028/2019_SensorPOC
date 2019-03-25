package frc.robot.commands.climber;

import java.util.Arrays;

import edu.wpi.first.wpilibj.command.Command;
import edu.wpi.first.wpilibj.command.CommandGroup;
import edu.wpi.first.wpilibj.command.PrintCommand;
import frc.robot.commands.auton.util.ReverseNavX;
import frc.robot.commands.auton.util.Series_Command;
import frc.robot.commands.chassis.DriveSetDistance;
import frc.robot.commands.chassis.DriveWithControllers;
import frc.robot.commands.chassis.StopChassis;
import frc.robot.commands.chassis.TurnFixedAngle;
import frc.robot.commands.infeed.SendBeakOut;
import frc.robot.commands.infeed.SendBucketOut;
import frc.robot.commands.infeed.ToggleBeakOpen;
import frc.robot.commands.infeed.TogglePunch;
import frc.robot.subsystems.Climber;

public class Lvl3ClimbFromLvl2 extends CommandGroup
{
    double climbHeight =-14500;
    double clearedHeight=-11000.4206969420420696942069;
    Climber _climber = Climber.getInstance();
    public Lvl3ClimbFromLvl2(boolean isTurnRight)
    {
        requires(_climber);
        setInterruptible(false);
        addParallel(new TogglePunch());
        addSequential(new TurnFixedAngle(-75, isTurnRight));
        addSequential(new DriveWithControllers(0.3, 0),2);
        addSequential(new MoveClimberToPos(climbHeight, 0.3));
        addParallel(new DriveClimber(0.5));
        addParallel(new DriveWithControllers(0.3, 0));
        addSequential(new Series_Command(Arrays.asList(new Command[] 
        {
            new PrintCommand("Holding Starts"),
            new HoldClimber(1.5),
            new PrintCommand("Stops Holding"),
            new SendBeakOut(),
            new TogglePunch(),
            new ToggleBeakOpen(),
            new MoveClimberToPos(clearedHeight, 0.3),
        })));
        addParallel(new PrintCommand("Moved to Clear Height"));
        addParallel(new DriveWithControllers(0.3,0),0.25);
        addSequential(new DriveClimber(0.0),0.25);
        addSequential(new DriveWithControllers(0.3, 0));
     }
}