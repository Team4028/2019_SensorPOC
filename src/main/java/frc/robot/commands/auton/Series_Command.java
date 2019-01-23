/* package frc.robot.commands;

import org.omg.CORBA.PRIVATE_MEMBER;
//#region  == Define Imports ==
import frc.robot.subsystems.Climber;
import frc.robot.subsystems.Elevator;
import java.util.ArrayList;
import java.util.List;

import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.command.Command;
//#endregion
import edu.wpi.first.wpilibj.command.CommandGroup;



public class Series_Command extends CommandGroup
{
    public Series_Command(List<Command> commands) {
      addParallel(new Auton_ParallelStarter());
		for (Command command : commands) {
            addSequential(command);
         }
		}

}

*/

package frc.robot.commands.auton;

//#region  == Define Imports ==
import java.util.ArrayList;
import java.util.List;

import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.command.Command;
//#endregion

/**
 * This command implements support for toggling the position of the Climber Servo
 */

public class Series_Command extends Command
{
    private Command _curCommand;
    private final ArrayList<Command> _remainingCommands;
    
    public Series_Command(List<Command> commands) {
    _remainingCommands = new ArrayList<>(commands.size()+1);
    _remainingCommands.add(new Auton_ParallelStarter());
		for (Command command : commands) {
			_remainingCommands.add(command);
			_curCommand = null; }
	}

    protected void initialize() {    
    }

    
    protected void execute() {
        if (_curCommand == null) {
            if (_remainingCommands.isEmpty()) {
                return;
            }
            _curCommand = _remainingCommands.remove(0);
            _curCommand.start();
        }
        //_curCommand.execute();
        if (_curCommand.isCompleted())
         {
            _curCommand.cancel();
            _curCommand = null;
        }
    }   

    protected boolean isFinished() {
        if (_remainingCommands.isEmpty() && _curCommand == null){
          System.out.println("Series Command Terminating");
          return true;
        } else {
          return false;
        }

    }

    protected void end() {
    }

    protected void interrupted() {
    }


}


