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
    double climbHeight =-15000;
    double clearedHeight=-11000.4206969420420696942069;
    Climber _climber = Climber.getInstance();
    public Lvl3ClimbFromLvl2(boolean isTurnRight)
    {
        requires(_climber);
        setInterruptible(false);
        if(isTurnRight)
        {
            addSequential(new TurnFixedAngle(75, isTurnRight));
        }
        else
        {
            addSequential(new TurnFixedAngle(-75, isTurnRight));
        }
        addSequential(new DriveWithControllers(0.2, 0),1);
        addSequential(new MoveClimberToPos(climbHeight,0.5));
        addParallel(new DriveClimber(0.5));
        addParallel(new DriveWithControllers(0.2, 0));
        addSequential(new Series_Command(Arrays.asList(new Command[] 
        {
            new HoldClimber(0.375),
            new MoveClimberToPos(climbHeight+2200, 0.2)
        })));
        addParallel(new DriveClimber(0.8));
        addSequential(new Series_Command(Arrays.asList(new Command[] 
        {
            new HoldClimber(2),
            new MoveClimberToPos(clearedHeight, 0.2),
            new HoldClimber(0.25)
        })));
        addParallel(new PrintCommand("Moved to Clear Height"));
        addSequential(new DriveWithControllers(0.3, 0),0.85);
        addSequential(new PrintCommand("Driven"));
        //addParallel(new StopChassis(),0.25);
        addSequential(new DriveClimber(0.0),0.5);
     }
}