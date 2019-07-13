
package frc.robot.commands.climber;

import java.util.Arrays;

import edu.wpi.first.wpilibj.command.Command;
import edu.wpi.first.wpilibj.command.CommandGroup;
import edu.wpi.first.wpilibj.command.PrintCommand;
import frc.robot.commands.auton.util.ReverseNavX;
import frc.robot.commands.auton.util.Series_Command;
import frc.robot.commands.chassis.DriveWithControllers;
import frc.robot.commands.chassis.StopChassis;
import frc.robot.commands.infeed.SendBeakOut;
import frc.robot.commands.infeed.SendBucketOut;
import frc.robot.commands.infeed.ToggleBeakInOut;
import frc.robot.commands.infeed.ToggleBeakOpen;
import frc.robot.commands.infeed.ToggleBeakOpenClose;
import frc.robot.commands.infeed.TogglePunch;
import frc.robot.subsystems.NEOChassis;
import frc.robot.subsystems.Climber;

public class Lvl3ClimbFromLvl2 extends CommandGroup
{
    double climbHeight =-15000;
    double clearedHeight=-11000.4206969420420696942069;
    Climber _climber = Climber.getInstance();
    NEOChassis _chassis = NEOChassis.getInstance();

    public Lvl3ClimbFromLvl2()
    {
        requires(_climber);
        setInterruptible(false);
        addSequential(new MoveClimberToPos(climbHeight,0.7));
            addParallel(new DriveClimber(0.5));
            addParallel(new DriveWithControllers(0.075, 0));
        addSequential(new HoldClimber(0.68));
            addParallel(new SendBucketOut());
            addParallel(new SendBeakOut());
            addParallel(new ToggleBeakOpenClose());
        addSequential(new MoveClimberToPos(climbHeight+3000, 0.2));
            addParallel(new DriveClimber(0.8));
            addParallel (new DriveWithControllers(0.40, 0),1.2);
        addSequential(new HoldClimber(2));// Make this 2 in push
        addSequential(new MoveClimberToPos(clearedHeight, 0.2));
        addSequential(new HoldClimber(0.25));
            addParallel(new PrintCommand("Moved to Clear Height"));
        addSequential(new DriveClimber(0.0),0.1);
        addSequential(new DriveWithControllers(0.20, 0),0.2);
        addSequential(new DriveWithControllers(0.05, 0),0.3);
        addSequential(new PrintCommand("Driven"));
        addSequential(new StopChassis(),0.25);
     }

}