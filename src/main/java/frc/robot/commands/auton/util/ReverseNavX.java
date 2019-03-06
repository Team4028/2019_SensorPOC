package frc.robot.commands.auton.util;

import edu.wpi.first.wpilibj.command.Command;
import frc.robot.sensors.GyroNavX;

public class ReverseNavX extends Command {

    boolean _isReversed;

    public ReverseNavX(boolean isReversed) {
        _isReversed = isReversed;
    }

    @Override
    protected void initialize() {
        GyroNavX.getInstance().setReversed(_isReversed);
    }

    @Override
    protected boolean isFinished() {
        return true;
    }
}