package frc.robot.commands.infeed;

import edu.wpi.first.wpilibj.command.Command;
import edu.wpi.first.wpilibj.smartdashboard.SendableBuilder;
import frc.robot.subsystems.Cargo;

public class RetainBall extends Command
{
    Cargo _cargo = Cargo.getInstance();
    public RetainBall()
    {
        setInterruptible(true);
        requires(_cargo);
    }
    @Override
    protected void initialize() {
        _cargo.setMotorSpeed(0.1);
    }
    @Override
    protected boolean isFinished() {
        return false;
    }
}