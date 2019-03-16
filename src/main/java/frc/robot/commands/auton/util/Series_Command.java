package frc.robot.commands.auton.util;

import java.util.ArrayList;
import java.util.List;
import edu.wpi.first.wpilibj.command.Command;

public class Series_Command extends Command {
    private Command _curCommand;
    private final ArrayList<Command> _remainingCommands;
    
    public Series_Command(List<Command> commands) {
        setInterruptible(false);
    _remainingCommands = new ArrayList<>(commands.size()+1);
    _remainingCommands.add(new Auton_ParallelStarter());
        for (Command command : commands) {
			_remainingCommands.add(command);
            _curCommand = null; 
        }
	}

    protected void initialize() {}

    protected void execute() {
        if (_curCommand == null) {
            if (_remainingCommands.isEmpty()) {
                return;
            }
            _curCommand = _remainingCommands.remove(0);
            _curCommand.start();
        }
        //_curCommand.execute();
        if (_curCommand.isCompleted()) {
            _curCommand.cancel();
            _curCommand = null;
        }
    }   

    protected boolean isFinished() {
        if (_remainingCommands.isEmpty() && _curCommand == null) {
          System.out.println("Series Command Terminating");
          return true;
        } else {
          return false;
        }
    }
}


