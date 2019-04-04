package frc.robot.commands.teleop;



import edu.wpi.first.wpilibj.buttons.Button;
import edu.wpi.first.wpilibj.command.Command;
import frc.robot.subsystems.Cargo;
import frc.robot.subsystems.Elevator;
import frc.robot.subsystems.Elevator.ELEVATOR_TARGET_POSITION;

public class StorePresetElevatorPosition extends Command
{
    Elevator _elevator = Elevator.getInstance();
    Cargo _cargo = Cargo.getInstance();
    Button _b,_x,_y, _rb;
    ELEVATOR_TARGET_POSITION _targetPos;
    public StorePresetElevatorPosition(Button b, Button x, Button y, Button rb)
    {
        setInterruptible(true);
        _b=b;
        _x=x;
        _y=y;
        _rb=rb;
    }
    @Override
    protected void execute() {
        if(_x.get())
        {
            _elevator.setTargetPosition(ELEVATOR_TARGET_POSITION.LEVEL_1, _cargo.get_isBeakOpen()); 
        }
        else if(_b.get())
        {
            _elevator.setTargetPosition(ELEVATOR_TARGET_POSITION.LEVEL_2, _cargo.get_isBeakOpen()); 
        }
        else if(_y.get())
        {
            _elevator.setTargetPosition(ELEVATOR_TARGET_POSITION.LEVEL_3, _cargo.get_isBeakOpen()); 
        }
        else if(_rb.get())
        {
            _elevator.setTargetPosition(ELEVATOR_TARGET_POSITION.HOME, _cargo.get_isBeakOpen());
        }              
    }
    @Override
    protected boolean isFinished() {
        return false;
    }
}