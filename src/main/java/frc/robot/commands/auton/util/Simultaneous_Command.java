package frc.robot.commands.auton.util;

import edu.wpi.first.wpilibj.command.Command;

import java.util.ArrayList;
import java.util.List;

/* Allows for multiple actions to run in parallel */
public class Simultaneous_Command extends Command{
	private ArrayList<Command> _commandList;
	private int _commandsNum;
	private int count = 0;
	boolean  isFirstCycle = true;

	public Simultaneous_Command(List<Command> commandList) {
		_commandList = new ArrayList<>(commandList.size()+1);
		_commandsNum = commandList.size();
		for (Command command : commandList) {
			_commandList.add(command);
		}
		setInterruptible(false);
	}
	
	protected void initialize() {
		System.out.println("Commands in Simultaneous: " + _commandList);
		System.out.println("number of Commands Read in simultaneous: " + _commandsNum);
	}

	protected void execute() {
		if (_commandList.isEmpty()){
			return;
		}
		if (isFirstCycle){
			for (int ind = 0; ind<_commandsNum; ind++) {
				_commandList.get(ind).start();	// Start all actions
			}
			isFirstCycle = false;
		}
		for (int ind = 0; ind<_commandsNum; ind++ ){
			if (_commandList.get(ind).isCompleted()){
				_commandList.get(ind).cancel();
				_commandList.remove(ind);
				_commandsNum--;
			}
		}
	}



	protected void end() {	
		
	}

	protected boolean isFinished() {	// Returns true when ALL actions are finished
		//System.out.println(" Running isFinished");
		if(_commandList.isEmpty()){
			System.out.println("FIRST TRY EVERY TIME EUREKA THE SIMULTANEOUS COMMAND HAS YIPPEE YEETED");
			return true;
		} else {
			//System.out.println("Be there Balm in gilead, nevermore, prophet, thing of evil be ye demon be ye devil");
			return false;
		}
	
	}

	protected void interrupted(){

	}

/*
	private boolean isAllTrue(List<Boolean> listOfBooles){
		if (count % 10 == 0 ) {
			System.out.println("BoolesList: " + listOfBooles);
		}
		int numBools = listOfBooles.size();
		for (int ind = 0; ind< numBools; ind++){
			if (!listOfBooles.get(ind)){
				return false;
			}
		}
		return true;
	}

	private List<Boolean> getHasFinishedList(ArrayList<Command> listOfCommands){
		List<Boolean> boolsList= new ArrayList<Boolean>();
		int numCommands = listOfCommands.size();
		for (int ind = 0; ind<numCommands; ind++){
			boolsList.add(listOfCommands.get(ind).isCompleted());
		}
		return boolsList;
	}
*/
}