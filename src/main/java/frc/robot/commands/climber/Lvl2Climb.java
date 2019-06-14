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
import frc.robot.commands.infeed.ToggleBeakOpen;
import frc.robot.commands.infeed.TogglePunch;
import frc.robot.subsystems.NEOChassis;
import frc.robot.subsystems.Climber;

public class Lvl2Climb extends CommandGroup
{
    double climbHeight =-8500;
    double clearedHeight=-696.9;//Nice
    Climber _climber = Climber.getInstance();
    NEOChassis _chassis = NEOChassis.getInstance();
    public Lvl2Climb()
    {
        requires(_climber);
        setInterruptible(false);
        addSequential(new MoveClimberToPos(climbHeight,0.5));
            addParallel(new DriveClimber(0.5));
            addParallel(new DriveWithControllers(0.2, 0));
        addSequential(new HoldClimber(0.6));
        addSequential(new MoveClimberToPos(climbHeight+1800, 0.2));
            addParallel(new DriveClimber(0.3));
        addSequential(new HoldClimber(.75));
        addSequential(new SendBeakOut());
        addSequential(new TogglePunch());
        addSequential(new ToggleBeakOpen());
        addSequential(new SendBucketOut());
        addSequential(new MoveClimberToPos(climbHeight+4000, 0.2));
        addSequential(new HoldClimber(.25));
        addSequential(new MoveClimberToPos(clearedHeight, 0.5));
            addParallel(new PrintCommand("Moved to Clear Height"));
        addSequential(new DriveWithControllers(0.3, 0),1.1);
        addSequential(new PrintCommand("Driven"));
            addParallel(new StopChassis(),0.25);
        addSequential(new DriveClimber(0.0),0.5);
        addSequential(new SendBucketIn());
        addSequential(new WaitCommand(0.1));
    }

}