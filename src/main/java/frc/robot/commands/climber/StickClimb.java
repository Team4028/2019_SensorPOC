package frc.robot.commands.climber;

import java.util.Arrays;

import edu.wpi.first.wpilibj.command.Command;
import edu.wpi.first.wpilibj.command.CommandGroup;
import edu.wpi.first.wpilibj.command.PrintCommand;
import edu.wpi.first.wpilibj.command.WaitCommand;
import edu.wpi.first.wpilibj.command.WaitUntilCommand;
import frc.robot.commands.auton.util.Series_Command;
import frc.robot.commands.auton.util.Simultaneous_Command;
import frc.robot.commands.chassis.DriveWithControllers;
import frc.robot.commands.chassis.StopChassis;
import frc.robot.commands.climber.Important.Secret.Hidden.Surprise.DontWorryAboutIt.EZWin.GGNoRE.YaYeetYaYeetYaYeetGirl.DasMyBurrito.IRICorn.VictorySpin;
import frc.robot.commands.infeed.ReleaseInfeed;
import frc.robot.commands.infeed.SendBeakOut;
import frc.robot.commands.infeed.SendBucketOut;
import frc.robot.commands.infeed.ToggleBeakInOut;
import frc.robot.commands.infeed.ToggleBeakOpen;
import frc.robot.commands.infeed.TogglePunch;
import frc.robot.subsystems.NEOChassis;
import frc.robot.subsystems.Climber;

public class StickClimb extends CommandGroup
{
    double climbHeight =-20500;
    double clearedHeight=-700;//15000?
    Climber _climber = Climber.getInstance();
    NEOChassis _chassis = NEOChassis.getInstance();

    public StickClimb()
    {
        requires(_climber);
        setInterruptible(false);
        //addParallel(new DriveClimber(0.2));
        addSequential(new MoveClimberToPos(climbHeight,0.5));
        addParallel(new DriveClimber(0.5));
        addParallel(new DriveWithControllers(0.05, 0));
        addSequential(new HoldClimber(0.575));
        addSequential(new MoveClimberToPos(climbHeight+1800, 0.2));
        addParallel(new DriveClimber(0.3));
        addSequential(new HoldClimber(.75));
        addSequential(new SendBeakOut());
        addSequential(new TogglePunch());
        addSequential(new ToggleBeakOpen());
        addSequential(new SendBucketOut());
        addSequential(new MoveClimberToPos(climbHeight+5000, 0.2));
        addParallel(new DriveWithControllers(0.075, 0));
        addParallel(new DriveClimber(0.0),0.5);
        addSequential(new HoldClimber(.75));
        // addSequential(new MoveClimberToPos(clearedHeight, 0.5)); 
        // addParallel(new PrintCommand("Moved to Clear Height"));
        // addSequential(new DriveWithControllers(0.3, 0),0.85);
        // addSequential(new PrintCommand("Driven"));
        addSequential(new DriveWithControllers(0.05, 0),0.25);
        addSequential(new DriveWithControllers(0.025, 0),0.25);
        addParallel(new StopChassis(),0.25);
        // addSequential(new VictorySpin(),3);

    }
}