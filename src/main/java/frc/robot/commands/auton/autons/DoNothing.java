package frc.robot.commands.auton.autons;
import edu.wpi.first.wpilibj.command.CommandGroup;

public class DoNothing extends CommandGroup {
    public DoNothing() {
        setInterruptible(true);
    }
    
    @Override
    protected boolean isFinished() {
        return true;
    }
}