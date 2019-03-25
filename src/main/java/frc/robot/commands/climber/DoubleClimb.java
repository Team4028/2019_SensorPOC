package frc.robot.commands.climber;
import edu.wpi.first.wpilibj.command.CommandGroup;
public class DoubleClimb extends CommandGroup{
    public DoubleClimb(boolean isTurnRight){
        addSequential(new Lvl2Climb());
        addSequential(new Lvl3ClimbFromLvl2(isTurnRight));
    }
}