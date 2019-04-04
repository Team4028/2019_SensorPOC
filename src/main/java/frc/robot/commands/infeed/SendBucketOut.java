package frc.robot.commands.infeed;


import edu.wpi.first.wpilibj.command.Command;
import frc.robot.subsystems.Cargo;

public class SendBucketOut extends Command
{
    Cargo _cargo = Cargo.getInstance();
    public SendBucketOut()
    {
        setInterruptible(false);
    }
    @Override
    protected void initialize() {
        if(!_cargo.get_isBucketExtended())
        {
            _cargo.toggleBucket();
        }
    }

    @Override
    protected boolean isFinished() {
        return true;
    }

}