// package frc.robot.sensors.visionLLInterpretation;

// import frc.robot.auton.pathfollowing.motion.RigidTransform;
// import frc.robot.sensors.GyroNavX;
// import frc.robot.sensors.VisionLL;
// import frc.robot.sensors.GyroNavX.SCORING_TARGET;
// import frc.robot.sensors.GyroNavX.SIDE;

// public class LimeLightInterpreter {

//     static GyroNavX _navX = GyroNavX.getInstance();
//     static VisionLL _limeLight = VisionLL.getInstance();

//     private static final double LIMELIGHT_X_OFFSET_INCHES = 0;    
//     private static final double LIMELIGHT_Y_OFFSET_INCHES = 0;    
//     private static final double LIMELIGHT_Z_OFFSET_INCHES = 0;
//     private static final double LIMELIGHT_ALPHA_OFFSET_DEGREES = 0; 
//     private static final double LIMELIGHT_BETA_OFFSET_DEGREES = 0;
//     private static final double LIMELIGHT_GAMMA_OFFSET_DEGREES = 0;
//     private static final double LIMELIGHT_ALPHA_OFFSET_RADIANS = LimeLightInterpreter.deg2rad(LIMELIGHT_ALPHA_OFFSET_DEGREES);
//     private static final double LIMELIGHT_BETA_OFFSET_RADIANS = LimeLightInterpreter.deg2rad(LIMELIGHT_BETA_OFFSET_DEGREES);
//     private static final double LIMELIGHT_GAMMA_OFFSET_RADIANS = LimeLightInterpreter.deg2rad(LIMELIGHT_GAMMA_OFFSET_DEGREES);

//     private static final ThreeDimensionalIsometry VEHICLE_TO_LIMELIGHT = new ThreeDimensionalIsometry(LIMELIGHT_X_OFFSET_INCHES, LIMELIGHT_Y_OFFSET_INCHES, LIMELIGHT_Z_OFFSET_INCHES, LIMELIGHT_ALPHA_OFFSET_RADIANS, LIMELIGHT_BETA_OFFSET_RADIANS, LIMELIGHT_GAMMA_OFFSET_RADIANS);

//     private static final double Y_DISTANCE_FROM_LIMELIGHT_TO_TARGET_HEIGHT_PLANE = 0;

//     private static LimeLightInformation limeLightInformation = LimeLightInformation.CannotSeeTarget();

//     public static double deg2rad(double deg){
//         return deg * Math.PI / 180;
//     }

//     public static double rad2deg(double rad){
//         return rad * 180 / Math.PI;
//     }

//     public static ThreeDimensionalIsometry translationFromSphericalCoordinates(double rho, double theta, double phi){
//         double x = rho * Math.cos(theta) * Math.sin(phi);
//         double y = rho * Math.sin(theta) * Math.sin(phi);
//         double z = rho * Math.cos(phi);
//         return new ThreeDimensionalIsometry(x, y, z, 0, 0, 0);
//     }

//     public static ThreeDimensionalIsometry translationFromAnglesAndFixedYDistance(double theta, double phi, double yDist){
//         double rho = yDist/(Math.cos(theta) * Math.sin(phi));
//         return LimeLightInterpreter.translationFromSphericalCoordinates(rho, theta, phi);
//     }

//     private static LimeLightInformation computeLimeLightDataFromRigidTransformation(RigidTransform rt, SCORING_TARGET target, SIDE side, double H, double a2){
//         double dx = rt.getTranslation().x(); 
//         double dy = rt.getTranslation().y(); 
//         double l = Math.sqrt(dx * dx + dy * dy);
//         double a1 = LimeLightInterpreter.deg2rad(Math.atan2(dy, dx)) - H;
//         double A2 = _navX.getTargetAngle(target, side) - a1 - H; 
//         return new LimeLightInformation(a1, A2, l);
//     }

//     public static void update(SCORING_TARGET target, SIDE side){
//         if (_limeLight.get_isTargetInFOV()){
//             double thetaVal = LimeLightInterpreter.deg2rad(_limeLight.getTheta());
//             double phiVal = LimeLightInterpreter.deg2rad(_limeLight.getPhi());
//             ThreeDimensionalIsometry limeLightToTargetTranslation = LimeLightInterpreter.translationFromAnglesAndFixedYDistance(thetaVal, phiVal, Y_DISTANCE_FROM_LIMELIGHT_TO_TARGET_HEIGHT_PLANE);
//             ThreeDimensionalIsometry limeLightToTargetRotation = _navX.getRotationToTarget(target, side);
//             ThreeDimensionalIsometry limeLightToTarget = limeLightToTargetRotation.applyIsometricTransformation(limeLightToTargetTranslation);
//             ThreeDimensionalIsometry  vehicleToTarget = VEHICLE_TO_LIMELIGHT.applyIsometricTransformation(limeLightToTarget);
//             RigidTransform vehicleToTargetAsRigidTransform = vehicleToTarget.getXZIsometry();
//             double A2 = _navX.get_angle2InDegreesFromLL(target, side); 
//             double H = _navX.getYaw();
//             limeLightInformation = LimeLightInterpreter.computeLimeLightDataFromRigidTransformation(vehicleToTargetAsRigidTransform, target, side, H, A2);
//         } else {
//             limeLightInformation = LimeLightInformation.CannotSeeTarget();
//         }
//     }

//     public static LimeLightInformation getLimeLightInfo(){
//         return LimeLightInterpreter.limeLightInformation;
//     }

//     public static double getDistanceToTargetInches(){
//         return LimeLightInterpreter.limeLightInformation.getLength();
//     }

//     public static double getAngleOneDegrees(){
//         return LimeLightInterpreter.limeLightInformation.getAngleOne();
//     }

//     public static double getAngleTwoDegrees(){
//         return LimeLightInterpreter.limeLightInformation.getAngleTwo();
//     }

//     public static boolean getIsTargetInFOV(){
//         return LimeLightInterpreter.limeLightInformation.canSeeTarget(); 
//     }

//     public static class LimeLightInformation{
//         private double angleOne; 
//         private double angleTwo; 
//         private double length;
//         private boolean canSeeTarget = false; 

//         public LimeLightInformation(double a1, double a2, double l){
//             angleOne = a1;
//             angleTwo = a2;
//             length = l;
//             canSeeTarget = true;
//         }

//         private LimeLightInformation(double a1, double a2, double l, boolean canSee){
//             angleOne = a1;
//             angleTwo = a2; 
//             length = l;
//             canSeeTarget = canSee;
//         }

//         public static LimeLightInformation CannotSeeTarget(){
//             return new LimeLightInformation(0, 0, 0, false); 
//         }


//         public double getLength(){
//             return length; 
//         }

//         public double getAngleOne(){
//             return angleOne; 
//         }

//         public double getAngleTwo(){
//             return angleTwo; 
//         }

//         public boolean canSeeTarget(){
//             return canSeeTarget; 
//         }
//     }
// }