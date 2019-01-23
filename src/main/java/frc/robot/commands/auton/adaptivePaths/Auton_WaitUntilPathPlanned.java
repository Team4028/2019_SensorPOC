package frc.robot.commands.auton.adaptivePaths;

import frc.robot.auton.path_planning.problem;
import edu.wpi.first.wpilibj.command.Command;

public class Auton_WaitUntilPathPlanned extends Command {
	
	public Auton_WaitUntilPathPlanned() {
	}
	
	@Override
	public void initialize() {}

	@Override
	public void execute() {}


	@Override
	public boolean isFinished() {
		return problem.pathPlanned;
	}
}