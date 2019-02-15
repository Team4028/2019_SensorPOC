package frc.robot.commands.auton.util;

import edu.wpi.first.wpilibj.command.Command;

public class Auton_ParallelStarter extends Command {
    String _printCmd;

    public Auton_ParallelStarter() {}

    @Override
    protected void initialize() {}
    
    @Override
    protected boolean isFinished() {
        System.out.println("Parallel Starter Has Finished");
        return true;
    }
}