package frc.robot.auton.path_planning;

import java.util.ArrayList;
import java.util.List;
import frc.robot.auton.pathfollowing.PathBuilder;
import frc.robot.auton.pathfollowing.PathBuilder.Waypoint;
import frc.robot.auton.pathfollowing.control.Path;
import frc.robot.auton.pathfollowing.motion.RigidTransform;
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

    public static void planPathFromVisionData(double A1, double A2, double l, RigidTransform curPose)
    {
        double k = (l*Math.sin(deg2rad(A2)))/Math.sin(deg2rad(180-A1-A2));
        if(k*Math.sin(curPose.getRotation().getRadians())> DISTANCE_OUT + 30 +curPose.getTranslation().y()+l*Math.sin(deg2rad(A1+curPose.getRotation().getDegrees())))
        {
            List<Waypoint> sWaypoints = new ArrayList<Waypoint>();
            sWaypoints.add(new Waypoint(curPose.getTranslation().x(),curPose.getTranslation().y(),0 ,0));
            sWaypoints.add(new Waypoint(curPose.getTranslation().x()+k*Math.cos(curPose.getRotation().getRadians()),curPose.getTranslation().y()+k*Math.sin(curPose.getRotation().getRadians()),20,VISION_PATH_DRIVE_SPEED));
            sWaypoints.add(new Waypoint(curPose.getTranslation().x()+l*Math.cos(deg2rad(A1+curPose.getRotation().getDegrees())), DISTANCE_OUT + curPose.getTranslation().y()+l*Math.sin(deg2rad(A1+curPose.getRotation().getDegrees())),0,VISION_PATH_DRIVE_SPEED));
            System.out.println(sWaypoints);
            System.out.println("Angle 1:"+A1);
            System.out.println("Angle2:"+ A2);
            System.out.println("Distance:"+l);
            _path = PathBuilder.buildPathFromWaypoints(sWaypoints);
            _theta = GyroNavX.getInstance().getYaw();
            _targetAngle = A1+A2+GyroNavX.getInstance().getYaw();
        }
        else
        {
            List<Waypoint> sWaypoints = new ArrayList<Waypoint>();
            sWaypoints.add(new Waypoint(curPose.getTranslation().x(),curPose.getTranslation().y(),0 ,0));
            sWaypoints.add(new Waypoint(curPose.getTranslation().x()+k*Math.cos(curPose.getRotation().getRadians()),DISTANCE_OUT + 31+curPose.getTranslation().y()+l*Math.sin(deg2rad(A1+curPose.getRotation().getDegrees())),20,VISION_PATH_DRIVE_SPEED));
            sWaypoints.add(new Waypoint(curPose.getTranslation().x()+l*Math.cos(deg2rad(A1+curPose.getRotation().getDegrees())), DISTANCE_OUT + curPose.getTranslation().y()+l*Math.sin(deg2rad(A1+curPose.getRotation().getDegrees())),0,VISION_PATH_DRIVE_SPEED));
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
