package frc.robot.auton.path_planning;

import java.util.ArrayList;
import java.util.List;
import frc.robot.auton.pathfollowing.PathBuilder;
import frc.robot.auton.pathfollowing.PathBuilder.Waypoint;
import frc.robot.auton.pathfollowing.control.Path;
import frc.robot.auton.pathfollowing.motion.RigidTransform;
import frc.robot.auton.pathfollowing.motion.Rotation;
import frc.robot.auton.pathfollowing.motion.Translation;
import frc.robot.sensors.GyroNavX;

public class problem{
    public static Path _path;
    public static double _theta;
    public static double _distance;
    public static double _targetAngle;

    private static final double DISTANCE_OUT = 45;
    private static final double VISION_PATH_DRIVE_SPEED = 70;

    public static double rad2deg(double rad){
        return rad * 180 / Math.PI;
    }
    public static double deg2rad(double deg){
        return deg*Math.PI /180 ;
    }
    public static double getDistance(Translation t1, Translation t2)
    {
        return Math.sqrt((t1.x() - t2.x()) * (t1.x() - t2.x()) + (t1.y() - t2.y()) * (t1.y() - t2.y()));
    }
    public static boolean isIntersectionInFrontofTarget(double intersectionYCoordinate, double targetYCoordinate)
    {
        return(intersectionYCoordinate>0 && targetYCoordinate>intersectionYCoordinate)||(intersectionYCoordinate<0 && targetYCoordinate<intersectionYCoordinate);
    }

    public static Translation getIntersect(RigidTransform startPose, RigidTransform endPose){
        double x0 = startPose.getTranslation().x();
        double y0 = startPose.getTranslation().y();
        double theta0 = startPose.getRotation().getRadians();
        double x1 = endPose.getTranslation().x();
        double y1 = endPose.getTranslation().y();
        double theta1 = endPose.getRotation().getRadians();
        double xt = ((y0 - y1 + x1 * Math.tan(theta1))/(Math.tan(theta1) - Math.tan(theta0)));
        double yt = (y0 + Math.tan(theta0)*(xt - x0));
        return new Translation(xt, yt);

    }

    public static void planPathFromVisionData(double A1, double A2, double l, RigidTransform curPose)
    {
        _targetAngle = A1+A2+curPose.getRotation().getDegrees();
        System.out.println("Target Angle:"+_targetAngle);
        double k = (l*Math.sin(deg2rad(A2)))/Math.sin(deg2rad(180-A1-A2));
        Translation endPoint = new Translation(curPose.getTranslation().x()+l*Math.cos(deg2rad(A1+curPose.getRotation().getDegrees())), curPose.getTranslation().y()+l*Math.sin(deg2rad(A1+curPose.getRotation().getDegrees())));
        RigidTransform endPose = new RigidTransform(endPoint, Rotation.fromDegrees(_targetAngle));
        Translation intersectionPoint = getIntersect(curPose, endPose);
        if(getDistance(intersectionPoint, endPoint)> DISTANCE_OUT + 30 && isIntersectionInFrontofTarget(intersectionPoint.y(), endPoint.y()))
        {
            List<Waypoint> sWaypoints = new ArrayList<Waypoint>();
            sWaypoints.add(new Waypoint(curPose.getTranslation().x(),curPose.getTranslation().y(),0 ,0));
            sWaypoints.add(new Waypoint(intersectionPoint.x(), intersectionPoint.y(),20,VISION_PATH_DRIVE_SPEED));
            sWaypoints.add(new Waypoint(endPoint.x()-DISTANCE_OUT*Math.cos(deg2rad(_targetAngle)),endPoint.y()-DISTANCE_OUT*Math.sin(deg2rad(_targetAngle)) ,0,VISION_PATH_DRIVE_SPEED));
            System.out.println(sWaypoints);
            System.out.println("Angle 1:"+A1);
            System.out.println("Angle2:"+ A2);
            System.out.println("Distance:"+l);
            _path = PathBuilder.buildPathFromWaypoints(sWaypoints);
            _theta = GyroNavX.getInstance().getYaw();
        }
        else
        {
            List<Waypoint> sWaypoints = new ArrayList<Waypoint>();
            sWaypoints.add(new Waypoint(curPose.getTranslation().x(),curPose.getTranslation().y(),0 ,0));
            sWaypoints.add(new Waypoint(endPoint.x()-(DISTANCE_OUT+30)*Math.cos(deg2rad(_targetAngle)),endPoint.y()-(DISTANCE_OUT+30)*Math.sin(deg2rad(_targetAngle)) ,20,VISION_PATH_DRIVE_SPEED));
            sWaypoints.add(new Waypoint(endPoint.x()-DISTANCE_OUT*Math.cos(deg2rad(_targetAngle)),endPoint.y()-DISTANCE_OUT*Math.sin(deg2rad(_targetAngle)) ,0,VISION_PATH_DRIVE_SPEED));
            System.out.println(sWaypoints);
            System.out.println("Angle 1:"+A1);
            System.out.println("Angle2:"+ A2);
            System.out.println("Distance:"+l);
            _path = PathBuilder.buildPathFromWaypoints(sWaypoints);
            _theta = 180/Math.PI*(Math.atan2(DISTANCE_OUT + 30 +l*Math.sin(deg2rad(A1+curPose.getRotation().getDegrees())),l*Math.cos(deg2rad(A1+curPose.getRotation().getDegrees()))));
        }
        System.out.println("Theta Turn:" + Double.toString(GyroNavX.getInstance().getYaw()-_theta));
    
    }

    public static void planSecondPathFromVisionData(double A1, double A2, double l, RigidTransform curPose)
    {
        /*
        List<Waypoint> sWaypoints = new ArrayList<Waypoint>();
        
        sWaypoints.add(new Waypoint(curPose.getTranslation().x(),curPose.getTranslation().y(),0 ,0));
        sWaypoints.add(new Waypoint(curPose.getTranslation().x()+l * Math.cos(a1 + H),28+curPose.getTranslation().y()+l*Math.sin(a1+H), 12 ,50));
        sWaypoints.add(new Waypoint(curPose.getTranslation().x()+l*Math.cos(a1+H),15+curPose.getTranslation().y()+l*Math.sin(a1+H),0,50));
        System.out.println(sWaypoints);
        

        System.out.println("Angle 1:"+A1);
        System.out.println("Distance:"+l);
        _path = PathBuilder.buildPathFromWaypoints(sWaypoints);
        
        */
        _distance=l;
        System.out.println("Distance:"+_distance);
        double a1 = deg2rad(A1);
        double H = deg2rad(curPose.getRotation().getDegrees());
        double unsignedTheta = Math.abs(180/Math.PI*(Math.atan2(l*Math.sin(a1 + H),l*Math.cos(a1 + H))));
        double targetAngle = A2+A1+GyroNavX.getInstance().getYaw();
        _theta = Math.copySign(unsignedTheta, targetAngle);
        System.out.println("TargetAngle:" + targetAngle);
        System.out.println("CurrentAngle:"+_theta);
        _path= PathBuilder.buildPathFromWaypoints(PathBuilder.getStraightPathWaypoints(curPose.getTranslation(), _theta, l-20));
    }
}
