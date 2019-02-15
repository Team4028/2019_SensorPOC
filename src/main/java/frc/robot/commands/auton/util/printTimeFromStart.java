package frc.robot.commands.auton.util;

import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj.command.Command;
import frc.robot.subsystems.Chassis;

public class printTimeFromStart extends Command {
    String _printCmd;

    public printTimeFromStart() {}

    @Override
    protected void initialize() {
        _printCmd = Double.toString(Timer.getFPGATimestamp() - Chassis._autoStartTime);
    }

    @Override
    protected boolean isFinished() {
        System.out.println(_printCmd);
        return true;
    }
}