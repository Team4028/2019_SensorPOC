package frc.robot.commands.auton.adaptivepaths;

import edu.wpi.first.wpilibj.command.Command;

public class FindTarget extends Command
{
    double conjecturedTarget;

    public FindTarget() {
    }

    @Override
    protected void initialize() {
        conjecturedTarget = problem.tar
    }

    @Override
    protected boolean isFinished() {
        System.out.println("Parallel Starter Has Finished");
        return true;
    }

}