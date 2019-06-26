package frc.robot.subsystems;

import com.revrobotics.CANSparkMax;
import com.revrobotics.CANSparkMax.IdleMode;
import com.revrobotics.CANSparkMaxLowLevel.MotorType;

import edu.wpi.first.wpilibj.command.Subsystem;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import frc.robot.RobotMap;
import frc.robot.interfaces.IBeakSquadSubsystem;
import frc.robot.util.LogDataBE;

public class NEOChassis extends Subsystem implements IBeakSquadSubsystem 
{
    CANSparkMax _leftMaster = new CANSparkMax(RobotMap.LEFT_DRIVE_MASTER_CAN_ADDR, MotorType.kBrushless);
    CANSparkMax _leftSlave = new CANSparkMax(RobotMap.LEFT_DRIVE_SLAVE_CAN_ADDR, MotorType.kBrushless);
    CANSparkMax _leftSlave2 = new CANSparkMax(RobotMap.LEFT_DRIVE_SLAVE2_CAN_ADDR, MotorType.kBrushless);
    CANSparkMax _rightMaster = new CANSparkMax(RobotMap.RIGHT_DRIVE_MASTER_CAN_ADDR, MotorType.kBrushless);
    CANSparkMax _rightSlave = new CANSparkMax(RobotMap.RIGHT_DRIVE_SLAVE_CAN_ADDR, MotorType.kBrushless);
    CANSparkMax _rightSlave2 = new CANSparkMax(RobotMap.RIGHT_DRIVE_SLAVE2_CAN_ADDR, MotorType.kBrushless);
    

    private static NEOChassis _instance = new NEOChassis();
    public static NEOChassis getInstance() 
    {
        return _instance;
    }
    public NEOChassis()
    {
        _leftSlave.follow(_leftMaster);
        _leftSlave2.follow(_leftMaster);
        _rightSlave.follow(_rightMaster);
        _rightSlave2.follow(_rightMaster);
        _rightMaster.setInverted(true);
        _rightSlave.setInverted(true);
        _rightSlave2.setInverted(true);
        _leftMaster.setInverted(false);
        _leftSlave.setInverted(false);
        _leftSlave2.setInverted(false);
        setIsBrakeMode(true);
        _leftMaster.setSmartCurrentLimit(40);
        _rightMaster.setSmartCurrentLimit(40);
        _leftSlave.setSmartCurrentLimit(40);
        _rightSlave.setSmartCurrentLimit(40);
        _leftSlave2.setSmartCurrentLimit(40);
        _rightSlave2.setSmartCurrentLimit(40);
        _rightMaster.burnFlash();
        _leftMaster.burnFlash();
        _leftSlave.burnFlash();
        _rightSlave.burnFlash();
        _rightSlave2.burnFlash();
        _leftSlave2.burnFlash();
    }

    public void setIsBrakeMode(boolean isBrakeMode)
    {
        if(isBrakeMode)
        {
            _leftMaster.setIdleMode(IdleMode.kBrake);
            _leftSlave.setIdleMode(IdleMode.kBrake);
            _rightMaster.setIdleMode(IdleMode.kBrake);
            _rightSlave.setIdleMode(IdleMode.kBrake);
            _rightSlave2.setIdleMode(IdleMode.kBrake);
            _leftSlave2.setIdleMode(IdleMode.kBrake);
        }
        else
        {
            _leftMaster.setIdleMode(IdleMode.kCoast);
            _leftSlave.setIdleMode(IdleMode.kCoast);
            _rightMaster.setIdleMode(IdleMode.kCoast);
            _rightSlave.setIdleMode(IdleMode.kCoast);
            _rightSlave2.setIdleMode(IdleMode.kCoast);
            _leftSlave2.setIdleMode(IdleMode.kCoast);
        }
    }

    public void setRampRate(double rampRate)
    {
        _leftMaster.setOpenLoopRampRate(rampRate);
        _leftSlave.setOpenLoopRampRate(rampRate);
        _rightMaster.setOpenLoopRampRate(rampRate);
        _rightSlave.setOpenLoopRampRate(rampRate);
        _leftSlave2.setOpenLoopRampRate(rampRate);
        _rightSlave2.setOpenLoopRampRate(rampRate);
    }

    public void arcadeDrive(double fwdCmd, double turnCmd)
    {
        if(turnCmd>0.3)
        {
            setRampRate(0.5);
        }
        else
        {
            setRampRate(0.85);
        }
        _leftMaster.set(0.75*fwdCmd+0.25*turnCmd);
        _rightMaster.set(0.75*fwdCmd-0.25*turnCmd);
    }
    public void elevatorUpArcadeDrive(double fwdCmd, double turnCmd)
    {
        if(turnCmd>0.3)
        {
            setRampRate(0.8);
        }
        else
        {
            setRampRate(1.0);
        }
        _leftMaster.set(0.5*fwdCmd+0.15*turnCmd);
        _rightMaster.set(0.5*fwdCmd-0.15*turnCmd);
    }

    public void stop()
    {
        _leftMaster.set(0);
        _rightMaster.set(0);
    }
    @Override
    public void updateLogData(LogDataBE logData) {

    }


    @Override
    public void updateDashboard() {
        SmartDashboard.putNumber("Chassis Current", _leftMaster.getOutputCurrent());

    }

    @Override
    protected void initDefaultCommand() {

	}

}