package frc.robot.commands.auton.adaptivePaths;

import frc.robot.auton.path_planning.problem;
import frc.robot.auton.pathfollowing.control.Path;
import edu.wpi.first.wpilibj.command.Command;

public class Auton_WaitUntilPathPlanned extends Command {

	Path p;
	
	public Auton_WaitUntilPathPlanned() {
	}
	
	@Override
	public void initialize() {
		p = problem.getPath();
	}


	@Override
	public void execute() {}


	@Override
	public boolean isFinished() {
		return true;
	}
}