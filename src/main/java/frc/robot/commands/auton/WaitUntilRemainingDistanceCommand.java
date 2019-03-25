package frc.robot.commands.auton;

import edu.wpi.first.wpilibj.command.Command;
import frc.robot.subsystems.Chassis;

public class WaitUntilRemainingDistanceCommand extends Command
{
    double _distance;
    Chassis _chassis = Chassis.getInstance();
    public WaitUntilRemainingDistanceCommand(double distance)
    {
        _distance=distance;
    }
    @Override
    protected boolean isFinished() {
        return _chassis.isDoneWithPath() || _chassis.getRemainingPathDistance()<_distance;
    }
}