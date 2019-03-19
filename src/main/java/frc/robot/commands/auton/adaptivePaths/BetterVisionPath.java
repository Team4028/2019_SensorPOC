package frc.robot.commands.auton.adaptivePaths;


import java.sql.Time;

import com.ctre.phoenix.motorcontrol.ControlMode;

import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj.buttons.Button;
import edu.wpi.first.wpilibj.command.Command;
import frc.robot.auton.pathfollowing.motion.RigidTransform;
import frc.robot.auton.pathfollowing.motion.Rotation;
import frc.robot.auton.pathfollowing.motion.Translation;
import frc.robot.auton.pathfollowing.motion.Twist;
import frc.robot.auton.pathfollowing.util.Kinematics;
import frc.robot.sensors.DistanceRev2mSensor;
import frc.robot.sensors.VisionLL;
import frc.robot.sensors.VisionLL.LIMELIGHT_PIPELINE;
import frc.robot.subsystems.Cargo;
import frc.robot.subsystems.Chassis;
import frc.robot.util.GeneralUtilities;

public class BetterVisionPath extends Command
{
    double kPa1, kPa2, dx, dy;
    double kDa1, kDa2;
    double prevDa1, prevDa2;
    double turnCmd, prevTurnCmd;
    double prevA1, currentTime, prevA2;
    double startTime;
    Chassis _chassis = Chassis.getInstance();
    VisionLL _limelight = VisionLL.getInstance();
    DistanceRev2mSensor _distanceSensor = DistanceRev2mSensor.getInstance();
    boolean isFirstCycle;
    boolean isForcedFinish;
    Button _a;
    int counter;
    double prevdx, prevdy;
    public static final double straightVBus = 0.25;
    public BetterVisionPath(Button a)
    {
        _a=a;
        setInterruptible(true);
        requires(_chassis);
    }
    @Override
    protected void initialize() 
    {
        counter=0;
        currentTime = Timer.getFPGATimestamp();
        startTime = Timer.getFPGATimestamp();
        prevA1=0;
        prevA2=0;
        dx=_limelight.get_xOffset();
        kPa1=0.007;
        kPa2=0.008;
        turnCmd=0;
        prevTurnCmd = 0;
        kDa1=0.001;
        kDa2=0.001;
        prevDa1=0;
        prevDa2=0;
        isFirstCycle=true;
        isForcedFinish = false;
        _limelight.changeLimelightPipeline(LIMELIGHT_PIPELINE.CENTER_PNP);
    }

    @Override
    protected void execute() {
        if(_limelight.get_isTargetInFOV())
        {
            double a1, a2, dA1, dA2;
            dx = _limelight.get_xOffset()+7;
            dy=_limelight.get_yOffset();
            a1 = _limelight.getTheta();
            a2 = Math.atan2(_limelight.get_xOffset()+7,_limelight.get_yOffset())*180/Math.PI;

            if(!isFirstCycle)
            {
                dA1 = 0.5* prevDa1 + 0.5*kDa1*(a1-prevA1)/(Timer.getFPGATimestamp()-currentTime);
                dA2 = 0.5 * prevDa2 + 0.5 *kDa2*(a2-prevA2)/(Timer.getFPGATimestamp()-currentTime);
                if(Math.abs(dA2)>0.1)
                {
                    dA2=0;
                }
            }
            else
            {
                dA1=0;
                dA2=0;
            }
            prevA1=a1;
            prevA2=a2;
            prevDa1=dA1;
            prevDa2=dA2;
            if(a2>90)
            {
                a2-=180;
            }
            if(a2<-90)
            {
                a2+=180;
            }
            turnCmd = (kPa2*a2+kPa1*a1+dA2+dA1);
            System.out.print("A1: "+GeneralUtilities.roundDouble(a1*kPa1, 4));
            System.out.print("            DA2: "+GeneralUtilities.roundDouble(dA2, 4));
            System.out.print("                A2: " + GeneralUtilities.roundDouble(a2*kPa2, 4));
            System.out.print(" dx: " + GeneralUtilities.roundDouble(dx, 4));
            System.out.println(" Turn: "+ GeneralUtilities.roundDouble(turnCmd, 4));
            _chassis.setLeftRightCommand(ControlMode.PercentOutput, straightVBus+turnCmd, straightVBus-turnCmd);
            prevTurnCmd = turnCmd;
            prevdx=dx;
            prevdy=dy;
        }
        else
        {
            
            // RigidTransform targetToVehicle = new RigidTransform(new Translation(prevdx, prevdy) , Rotation.fromDegrees(prevA1 + prevA2));
            // Twist odometry = Kinematics.forwardKinematics(_chassis.getLeftVelocityInchesPerSec(), _chassis.getRightVelocityInchesPerSec());
            // RigidTransform newTargetToVehicle = Kinematics.integrateForwardKinematics(targetToVehicle, odometry);
            // double dxPrime = newTargetToVehicle.getTranslation().x();
            // double a2Prime = Math.atan2(7+newTargetToVehicle.getTranslation().x(), newTargetToVehicle.getTranslation().y()) * 180 / Math.PI;
            // double a1Prime = newTargetToVehicle.getRotation().getDegrees() - a2Prime;
            // turnCmd = kPa2*a2Prime+kPa1*a1Prime;

            System.out.println("Lost LL");
            if(Math.abs(turnCmd)<0.04 && !isFirstCycle)
            {
                _chassis.setLeftRightCommand(ControlMode.PercentOutput, straightVBus, straightVBus);
            }
            else if(isFirstCycle)
            {
                System.out.println("Cannot See Target");
                isForcedFinish = true;
            }
            else if(!isFirstCycle && Math.abs(turnCmd)>=0.04)
            {
                System.out.println("Angle of Approach is Too Aggressive. Also, Ciarn Says You Are Bad and I am Right - Nick :P <3");
                isForcedFinish = true;
            }
        }
        if(isFirstCycle)
        {
            isFirstCycle = false;
        }
        currentTime = Timer.getFPGATimestamp();
    }

    @Override
    protected boolean isFinished() {
        boolean isFin =  ((!_a.get()) || (_distanceSensor.get_distanceToTargetInInches()>0 && _distanceSensor.get_distanceToTargetInInches()<23)|| isForcedFinish);
        if(isFin)
        {
            System.out.print("Button: "+ !_a.get());
            System.out.print(" Distance: " + (_distanceSensor.get_distanceToTargetInInches()>0 && _distanceSensor.get_distanceToTargetInInches()<23));
            System.out.println(" Force Finished: "+ isForcedFinish);
            _chassis.stop();
            counter++;
            
        }
        else
        {
            counter=0;
        }
        return counter>=4|| Math.abs(turnCmd)>0.4;
    }

    @Override
    protected void end() {
        _chassis.stop();
        System.out.println("END RUNNING");
    }
    @Override
    protected void interrupted() {
        _chassis.stop();
        System.out.println("INTERRUPTED");
    }
}