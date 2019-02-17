package frc.robot.auton.pathfollowing;

import frc.robot.auton.pathfollowing.motion.RigidTransform;
import frc.robot.auton.pathfollowing.motion.Rotation;
import frc.robot.auton.pathfollowing.motion.Translation;
import frc.robot.sensors.DistanceRev2mSensor;
import frc.robot.sensors.GyroNavX;
import frc.robot.sensors.VisionLL;
import frc.robot.sensors.GyroNavX.SCORING_TARGET;
import frc.robot.sensors.GyroNavX.SIDE;

public class LimeLightInterpreter{

    private static final GyroNavX _navX = GyroNavX.getInstance();
    private static final VisionLL _limeLight = VisionLL.getInstance();
    private static final DistanceRev2mSensor _distanceSensor = DistanceRev2mSensor.getInstance();
    
    private static LimeLightInformation limeLightInfo = LimeLightInformation.CannotSeeTarget();

    private static final double LIMELIGHT_X_OFFSET = 0;
    private static final double LIMELIGHT_Y_OFFSET = 0;
    private static final double LIMELIGHT_THETA_OFFSET = 0;
    private static final RigidTransform VEHICLE_TO_LIMELIGHT = new RigidTransform(new Translation(LIMELIGHT_X_OFFSET, LIMELIGHT_Y_OFFSET), Rotation.fromDegrees(LIMELIGHT_THETA_OFFSET));

    private static RigidTransform computeRigidTransformFromLimeLightToTarget(double A1, double A2, double H, double l, SCORING_TARGET target, SIDE side){
        double a1 = A1 * Math.PI / 180;
        double h = H * Math.PI/180;
        double targetAngle = _navX.getTargetAngle(target, side) * Math.PI/180;
        double dx = l * Math.cos(a1 + h);
        double dy = l * Math.sin(a1 + h);
        double dTheta = targetAngle - a1 - h;
        RigidTransform limeLightToTarget = new RigidTransform(new Translation(dx, dy), Rotation.fromRadians(dTheta));
        return limeLightToTarget;
    }

    private static LimeLightInformation computeLimeLightDataFromRigidTransformation(RigidTransform rt, SCORING_TARGET target, SIDE side, double H, double a2){
        double dx = rt.getTranslation().x();
        double dy = rt.getTranslation().y();
        double l = Math.sqrt(dx * dx + dy * dy);
        double a1 = Math.atan2(dy, dx) * 180 / Math.PI - H;
        // double a1 = _navX.getTargetAngle(target, side) - rt.getRotation().getDegrees() - H;
        return new LimeLightInformation(a1, a2, l);
    }

    private static RigidTransform computeVehicleToTargetFromLimeLightToTarget(RigidTransform limeLightToTarget){
        return VEHICLE_TO_LIMELIGHT.transformBy(limeLightToTarget);
    }

    public static void update(SCORING_TARGET target, SIDE side){
        if(! _limeLight.get_isTargetInFOV()){
            limeLightInfo = LimeLightInformation.CannotSeeTarget();
        } else {
            double l = _limeLight.get_distanceToTargetInInches();
            double A1 = _limeLight.get_angle1InDegrees();
            double A2 = _navX.get_angle2InDegreesFromLL(target, side);
            double H = _navX.getYaw();
            RigidTransform limeLightToTarget = LimeLightInterpreter.computeRigidTransformFromLimeLightToTarget(A1, A2, H, l, target, side);
            RigidTransform vehicleToTarget = LimeLightInterpreter.computeVehicleToTargetFromLimeLightToTarget(limeLightToTarget);
            LimeLightInformation limeLightInformation = LimeLightInterpreter.computeLimeLightDataFromRigidTransformation(vehicleToTarget, target, side, H, A2);
            limeLightInfo = limeLightInformation;
        }
    }

    public static LimeLightInformation getLimeLightInfo(){
        return LimeLightInterpreter.limeLightInfo;
    }

    public static double getDistanceToTargetInches(){
        return LimeLightInterpreter.limeLightInfo.getLength();
    }

    public static double getAngleOneDegrees(){
        return LimeLightInterpreter.limeLightInfo.getAngleOne();
    }

    public static double getAngleTwoDegrees(){
        return LimeLightInterpreter.limeLightInfo.getAngleTwo();
    }

    public static boolean getIsTargetInFOV(){
        return LimeLightInterpreter.limeLightInfo.canSeeTarget();
    }

    public static class LimeLightInformation{
        private double angleOne;
        private double angleTwo;
        private double length;
        private boolean canSeeTarget = false;

        public LimeLightInformation(double a1, double a2, double l){
            angleOne = a1;
            angleTwo = a2;
            length = l;  
            canSeeTarget = true;          
        }

        private LimeLightInformation(double a1, double a2, double l, boolean canSee){
            angleOne = a1;
            angleTwo = a2;
            length = l;  
            canSeeTarget = canSee;   
        }

        public static LimeLightInformation CannotSeeTarget(){
            return new LimeLightInformation(0, 0, 0, false);
        }

        public double getLength(){
            return length;
        }

        public double getAngleOne(){
            return angleOne;
        }

        public double getAngleTwo(){
            return angleTwo;
        }

        public boolean canSeeTarget(){
            return canSeeTarget;
        }
    }
}