package frc.robot.commands.climber.Important.Secret.Hidden.Surprise.DontWorryAboutIt.EZWin.GGNoRE.YaYeetYaYeetYaYeetGirl.DasMyBurrito.IRICorn;

import com.ctre.phoenix.motorcontrol.ControlMode;

import edu.wpi.first.wpilibj.command.Command;
import frc.robot.subsystems.Chassis;

public class VictorySpin extends Command
{
    Chassis _chassis = Chassis.getInstance();
    public VictorySpin()
    {
        requires(_chassis);
        setInterruptible(true);
    }
    @Override
    protected void execute() {
        _chassis.setLeftRightCommand(ControlMode.PercentOutput, 0.5, -0.5);
    }
	@Override
	protected boolean isFinished() {
		return false;
    }
    @Override
    protected void end() {
        _chassis.stop();
    }
}
