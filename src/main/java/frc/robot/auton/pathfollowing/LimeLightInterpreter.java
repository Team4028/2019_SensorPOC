package frc.robot.auton.pathfollowing;

/////////// Imports //////////////////////////////////////////
import frc.robot.auton.pathfollowing.motion.RigidTransform;
import frc.robot.auton.pathfollowing.motion.Rotation;
import frc.robot.auton.pathfollowing.motion.Translation;
import frc.robot.sensors.DistanceRev2mSensor;
import frc.robot.sensors.GyroNavX;
import frc.robot.sensors.VisionLL;
import frc.robot.sensors.GyroNavX.SCORING_TARGET;
import frc.robot.sensors.GyroNavX.SIDE;
//////////////////////////////////////////////////////////////


/** This is a completely static class with one private gettable and updatable globabl variable containing the information of a virtual on center limelight computed from the off center limelight information
 * */ 
public class LimeLightInterpreter{

    // Instance Variables of Vision Classes
    private static final GyroNavX _navX = GyroNavX.getInstance(); //local instance of the navX for angle data and scoring target based target angle handling
    private static final VisionLL _limeLight = VisionLL.getInstance(); //local instance of the limelight to get its vision data
    
    //Global Limelight Data Variable to get and set to avoid updating to call for multiple measurements
    private static LimeLightInformation limeLightInfo = LimeLightInformation.CannotSeeTarget(); //Initialized to not see for safe handling in case of off-book use

    //Physicsal Constants relating the Pose of the LimeLight to the Posiion of The Robot's Center Of Mass
    private static final double LIMELIGHT_X_OFFSET = 0;     //The X Offset Between the Limelight and the Robot's Center of Mass in Inches
    private static final double LIMELIGHT_Y_OFFSET = 0;     //The Y Offset Between the Limelight and the Robot's Center of Mass in Inches
    private static final double LIMELIGHT_THETA_OFFSET = 0; //The Angle Offset Between the Forward Axis of The Chassis and The Projection of the Limelight's Pinhole into the X-Y Plane (That is, the plane of the field) 
    private static final RigidTransform VEHICLE_TO_LIMELIGHT = new RigidTransform(new Translation(LIMELIGHT_X_OFFSET, LIMELIGHT_Y_OFFSET), Rotation.fromDegrees(LIMELIGHT_THETA_OFFSET)); //A RigidTransform object encompassing the Isometric Transformation from the Robot's center of mass angled at it's driving axis to the Camera Position of the Limelight With The Projection pinhol of the limelight to the Plane of the Field

    /**
     * This Method Takes in limelight result data along with information about the scoring target and the current Gyro heading
     * and returns the Rigid Transformation Computed Between The Limelight and The Target Pose. More explicitly it may be 
     * considered as computing the changes in x, y and theta between the limelight pose and the target pose.  
     * @param  A1       {@link Double}; The angle between the Limelight's current heading and the heading of a strait path between the limelight and it's target in Degrees
     * @param  A2       {@link Double}; The angle between the extension of the target angle through the target pose and the strait line between the limelight and the target translation 
     * @param  H        {@link Double}; The Current Heading of the Robot, relative to its starting angle of 0, in degrees
     * @param  L        {@link Double}; The Distance between the Limelight and the Target, in inches
     * @param side      {@link frc.robot.sensors.GyroNavX.SIDE}; The side of the field on which scoring is intended, used to compute target Angle
     * @param target    {@link frc.robot.sensors.GyroNavX.SCORING_TARGET}; The Object Upon Which Scoring is Attempted
     * @return          {@link RigidTransform}; The Isometric Transformation From The Limelight to The Target Pose
     * @see         See https://www.chiefdelphi.com/uploads/default/original/3X/5/e/5e2fb8ce56e27d7f4f63a97731700e4684d550e4.pdf for definition of Angle One, Angle Two, and L
    */

    private static RigidTransform computeRigidTransformFromLimeLightToTarget(double A1, double A2, double H, double l, SCORING_TARGET target, SIDE side){
        double a1 = A1 * Math.PI / 180; //Rewrite angle one in raadians
        double h = H * Math.PI/180; //Rewrite Current heading in radians
        double targetAngle = _navX.getTargetAngle(target, side) * Math.PI/180; //Use the information on what place and what target we score on to compute the target angle of the Robot's final pose with respect to it's angle at the begining of the game in degrees, and then convert it to radians
        double dx = l * Math.cos(a1 + h); //Compute the difference in X position between the limelight and the target in inches
        double dy = l * Math.sin(a1 + h); //Compute the difference in Y position between the limelight and the target in inches
        double dTheta = targetAngle - a1 - h; //Compute the difference in Angle Between the Limelight and the target in radians
        RigidTransform limeLightToTarget = new RigidTransform(new Translation(dx, dy), Rotation.fromRadians(dTheta)); //Construct a RigidTransform Object with the proper x, y, and theta change to map the limelight onto the target
        return limeLightToTarget; //return the rigid transform between the limelight and the target
    }

    /**
     * This Mehtod is the inverse of {@link computeRigidTransformFromLimeLightToTarget}. Given the RigidTransform between a pose and the target pose, it will compute the 
     * LimelightInformation that the limelight would measure. This allows us to transform the limelight data directly and use it instead of some of it's useful derivatives
     * @param  rt       {@link RigidTransform}; Isometric Transformation Mapping the Vehicle to the Target
     * @param target    {@link frc.robot.sensors.GyroNavX.SCORING_TARGET}; The Object Upon Which Scoring is Attempted
     * @param side      {@link frc.robot.sensors.GyroNavX.SIDE}; The side of the field on which scoring is intended, used to compute target Angle     
     * @param  H        {@link Double}; The Current Heading of the Robot, relative to its starting angle of 0, in degrees     
     * @param  A2       {@link Double}; The angle between the extension of the target angle through the target pose and the strait line between the limelight and the target translation 
     * @return          {@link LimeLightInformation}; The Limelight information if the limelight were located at the Robot's center of mass in the direction of forward chassis movement
     * @see         See https://www.chiefdelphi.com/uploads/default/original/3X/5/e/5e2fb8ce56e27d7f4f63a97731700e4684d550e4.pdf for definition of Angle One, Angle Two, and L
    */

    private static LimeLightInformation computeLimeLightDataFromRigidTransformation(RigidTransform rt, SCORING_TARGET target, SIDE side, double H, double a2){
        double dx = rt.getTranslation().x(); //Isolate the change in X Position from the Robot's Center of mass to the target
        double dy = rt.getTranslation().y(); //Isolate the change in Y Position from the Robot's Center of mass to the target
        double l = Math.sqrt(dx * dx + dy * dy); //Compute The Distance Between the current Translation (XY position) and the target Translation
        double a1 = Math.atan2(dy, dx) * 180 / Math.PI - H; //Compute what would be angle one in degrees. From the inverse of computation of dx and dy above
        double A2 = _navX.getTargetAngle(target, side) - a1 - H; //Compute the new angle two in degrees. This is identical to _navX.getAngle2InDegreesFromLL, but using the just computed virtual angle 1
        return new LimeLightInformation(a1, A2, l); //Return the Limelight Information that would be observed if the limelight were at the robot's center of Mass
    }

    /**
     * This Method takes in the RigidTransformation between the limelight and the target and returns the RigidTransformation between the Vehicle and the Target
     * @param  limeLightToTarget   {@link RigidTransform}; The isometric transformation mapping the 
     * @return                     {@link LimeLightInformation}; The Limelight information if the limelight were located at the Robot's center of mass in the direction of forward chassis movement
     * @see                    See {@link RigidTransform} for an explanation of the transformation
    */ 

    private static RigidTransform computeVehicleToTargetFromLimeLightToTarget(RigidTransform limeLightToTarget){
        return VEHICLE_TO_LIMELIGHT.transformBy(limeLightToTarget); //Compute and return the isometry mapping the vehicle to the target as the isometry that is the composition of the isometry mapping the vehicle to the limelight with the isometry mapping the limelight to the target.
    }

    /**
     * This Method uses information about the scoring target to cpmpute target angle along with updated Limelight data to compute the limelight data we would recieve from an on-center limelight. 
     * @param target    {@link frc.robot.sensors.GyroNavX.SCORING_TARGET}; The Object Upon Which Scoring is Attempted
     * @param side      {@link frc.robot.sensors.GyroNavX.SIDE}; The side of the field on which scoring is intended, used to compute target Angle   
     * @return          {@link Void}
     * @see         See https://www.chiefdelphi.com/uploads/default/original/3X/5/e/5e2fb8ce56e27d7f4f63a97731700e4684d550e4.pdf for definition of Vision Data
    */ 

    public static void update(SCORING_TARGET target, SIDE side){
        if(! _limeLight.get_isTargetInFOV()){
            limeLightInfo = LimeLightInformation.CannotSeeTarget(); //if the limelight cannot see the target, make not seeing the target the current limeLight Information
        } else {
            double l = _limeLight.get_distanceToTargetInInches(); //If the limelight can see the target, isolate the current limelight distance in inches
            double A1 = _limeLight.get_angle1InDegrees(); //Isolate the current limelight angle one in degrees
            double A2 = _navX.get_angle2InDegreesFromLL(target, side); //Isolate the current angle two in degrees from the limelight and the scoring information
            double H = _navX.getYaw(); //Isolate the current heading in degrees from the navX
            RigidTransform limeLightToTarget = LimeLightInterpreter.computeRigidTransformFromLimeLightToTarget(A1, A2, H, l, target, side); //Compute the Rigid Transformation mapping the limelight to the target from the limelight data
            RigidTransform vehicleToTarget = LimeLightInterpreter.computeVehicleToTargetFromLimeLightToTarget(limeLightToTarget); //Compute the rigid transformation mapping the vehicle to the target from limelight 
            LimeLightInformation limeLightInformation = LimeLightInterpreter.computeLimeLightDataFromRigidTransformation(vehicleToTarget, target, side, H, A2); //Compute the Limelight Info resulting from the rigid transformation from the vehicle to the limelight
            limeLightInfo = limeLightInformation; //set the global static information to that computed above. Allows for specific pieces of the information to be queried multiple times without repeating computation
        }
    }
    
    /** 
     * A Simple getter method for the limelight information object. Should be called only after updating
     * @return      {@link LimeLightInformation}; the limelight information object of the virtual limelight with the same pinhole as the chassis
     * @see         See https://www.chiefdelphi.com/uploads/default/original/3X/5/e/5e2fb8ce56e27d7f4f63a97731700e4684d550e4.pdf for definition of Vision Data
     */

    public static LimeLightInformation getLimeLightInfo(){
        return LimeLightInterpreter.limeLightInfo; //return the limelight info
    }
    
    /** 
     * A Simple getter method for the limelight computed distance to target. Should be called only after updating
     * @return      {@link Double}; the distance from the center of mass of the chassis to the target in inches
     * @see         See https://www.chiefdelphi.com/uploads/default/original/3X/5/e/5e2fb8ce56e27d7f4f63a97731700e4684d550e4.pdf for definition of this length    
     */

    public static double getDistanceToTargetInches(){
        return LimeLightInterpreter.limeLightInfo.getLength(); //return the length to the target
    }

    /** 
     * A Simple getter method for the limelight computed Angle One To The Target In Degrees. Should be called only after updating
     * @return      {@link Double}; The angle between the Limelight's current heading and the heading of a strait path between the limelight and it's target in Degrees
     * @see         See https://www.chiefdelphi.com/uploads/default/original/3X/5/e/5e2fb8ce56e27d7f4f63a97731700e4684d550e4.pdf for definition of this angle one
    */

    public static double getAngleOneDegrees(){
        return LimeLightInterpreter.limeLightInfo.getAngleOne(); //return angle one in degrees
    }

    /** 
     * A Simple getter method for the limelight computed Angle Two To The Target In Degrees. Should be called only after updating
     * @return      {@link Double}; The angle between the extension of the target angle through the target pose and the strait line between the limelight and the target translation 
     * @see         See https://www.chiefdelphi.com/uploads/default/original/3X/5/e/5e2fb8ce56e27d7f4f63a97731700e4684d550e4.pdf for definition of this angle one
    */

    public static double getAngleTwoDegrees(){
        return LimeLightInterpreter.limeLightInfo.getAngleTwo(); //return angle two in degrees
    }

    /** 
     * A Simple getter method for whether the limelight saw the target the last time update was called. Should be called only after updating
     * @return      {@link Boolean}; whether or not the limelight saw the target on the last update cycle
    */

    public static boolean getIsTargetInFOV(){
        return LimeLightInterpreter.limeLightInfo.canSeeTarget(); //return whether the limelight saw the target
    }

    /**
     *          This is a simple class encompassing angle one, angle two, and theta, most used for compact casing of this information
     * @see See https://www.chiefdelphi.com/uploads/default/original/3X/5/e/5e2fb8ce56e27d7f4f63a97731700e4684d550e4.pdf for definition Vision Data
    */
    public static class LimeLightInformation{
        private double angleOne; //The angle between the Limelight's current heading and the heading of a strait path between the limelight and it's target in Degrees
        private double angleTwo; //The angle between the extension of the target angle through the target pose and the strait line between the limelight and the target translation 
        private double length; //The Distance between the Limelight and the Target, in inches
        private boolean canSeeTarget = false; //Whether or not the limelight saw the target. Initialized to false for intuitive stability

        /**
         * Constructs a LimeLight Information object from the specifics of the information. Should ony be constructed if target is seen
         * 
         * @param  a1       {@link Double}; The angle between the Limelight's current heading and the heading of a strait path between the limelight and it's target in Degrees
         * @param  a2       {@link Double}; The angle between the extension of the target angle through the target pose and the strait line between the limelight and the target translation 
         * @param  l        {@link Double}; The Distance between the Limelight and the Target, in inches
        */

        public LimeLightInformation(double a1, double a2, double l){
            angleOne = a1; //set the angle one of the information
            angleTwo = a2; //set the angle two of the information
            length = l; //set the distance of the information
            canSeeTarget = true;  //update the assumption that the target was seen
        }

        /**
         * Constructs a LimeLight Information object from the specifics of the information with manual setting of the canSee boolean. Private Method only used to construct the public cantSee Information
         * 
         * @param  a1       {@link Double}; The angle between the Limelight's current heading and the heading of a strait path between the limelight and it's target in Degrees
         * @param  a2       {@link Double}; The angle between the extension of the target angle through the target pose and the strait line between the limelight and the target translation 
         * @param  l        {@link Double}; The Distance between the Limelight and the Target, in inches
         * @param canSee    {@link Boolean}; Wheteher the limelight can see the target
        */

        private LimeLightInformation(double a1, double a2, double l, boolean canSee){
            angleOne = a1; //set the angle one of the information
            angleTwo = a2; //set the angle two of the transformation
            length = l;   //set the distance of the information
            canSeeTarget = canSee; //set whether the limelight can see the target
        }

        /**
         * A simple getter method for the infprmation that the limelight cannot see the target
         * @return {@link LimeLightInformation}the limelight information that the limelight cannot see the target
         */

        public static LimeLightInformation CannotSeeTarget(){
            return new LimeLightInformation(0, 0, 0, false); //return a new information with the target equal to the current position for stability and the canSee boolean set to false
        }

        /** 
         * A Simple getter method for the distance to the target in inches
         * @return      {@link Double}; the distance from the limelight to the target in inches
         * @see         See https://www.chiefdelphi.com/uploads/default/original/3X/5/e/5e2fb8ce56e27d7f4f63a97731700e4684d550e4.pdf for definition of this length    
        */

        public double getLength(){
            return length; //return the length of the segment from the limelight to the target
        }

        /** 
         * A Simple getter method for the Angle One To The Target In Degrees
         * @return      {@link Double}; The angle between the Limelight's current heading and the heading of a strait path between the limelight and it's target in Degrees
         * @see         See https://www.chiefdelphi.com/uploads/default/original/3X/5/e/5e2fb8ce56e27d7f4f63a97731700e4684d550e4.pdf for definition of this angle one
        */

        public double getAngleOne(){
            return angleOne; //return angle one
        }

        /** 
         * A Simple getter method for the Angle Two To The Target In Degrees
         * @return      {@link Double}; The angle between the extension of the target angle through the target pose and the strait line between the limelight and the target translation 
         * @see         See https://www.chiefdelphi.com/uploads/default/original/3X/5/e/5e2fb8ce56e27d7f4f63a97731700e4684d550e4.pdf for definition of this angle one
        */

        public double getAngleTwo(){
            return angleTwo; //return angle two
        }

        /** 
         * A Simple getter method for whether the limelight saw the target
         * @return      {@link Boolean}; whether or not the limelight saw the target
        */

        public boolean canSeeTarget(){
            return canSeeTarget; //return wheter or not the limelight saw the target
        }
    }
}