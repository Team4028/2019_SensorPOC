package frc.robot.commands.auton.adaptivePaths;

import frc.robot.auton.pathfollowing.RobotState;
import frc.robot.auton.pathfollowing.motion.RigidTransform;
import frc.robot.auton.pathfollowing.motion.Rotation;
import frc.robot.auton.pathfollowing.motion.Translation;
import frc.robot.sensors.GyroNavX;
import frc.robot.subsystems.Chassis;
import edu.wpi.first.wpilibj.command.Command;

public class resetRobotPose extends Command
{
    GyroNavX _navX = GyroNavX.getInstance();
    public resetRobotPose() {
    }

    @Override
    protected void initialize() {
        RobotState.getInstance().reset(0, new RigidTransform(new Translation(), Rotation.fromDegrees(_navX.getYaw())));
    }

    @Override
    protected boolean isFinished() {
        return true;
    }

}