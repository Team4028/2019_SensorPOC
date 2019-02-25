package frc.robot.commands.auton.util;

import edu.wpi.first.wpilibj.command.Command;

public class PrintCommand extends Command {
    String _printCmd;

    public PrintCommand(String printCmd) {
        _printCmd=printCmd;
    }

    @Override
    protected void initialize() {
        System.out.println(_printCmd);
    }

    @Override
    protected boolean isFinished() {
        return true;
    }
}