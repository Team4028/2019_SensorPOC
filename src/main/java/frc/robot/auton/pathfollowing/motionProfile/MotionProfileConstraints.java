package frc.robot.auton.pathfollowing.motionProfile;

/** Constraints for constructing a MotionProfile */
public class MotionProfileConstraints {
	protected double maxAbsVel = Double.POSITIVE_INFINITY;
	protected double maxAcc = Double.POSITIVE_INFINITY;
	protected double maxDecel = Double.POSITIVE_INFINITY;
	
	public MotionProfileConstraints(double max_vel, double max_acc, double max_decel) {
		this.maxAcc = max_acc;
		this.maxAbsVel = Math.abs(max_vel);
		this.maxDecel = max_decel;
	}
}