package frc.robot.auton.path_planning.math;

import java.lang.Math;

import frc.robot.auton.path_planning.problem;
import frc.robot.auton.path_planning.math.point;
import frc.robot.auton.pathfollowing.motion.RigidTransform;
import frc.robot.auton.pathfollowing.motion.Rotation;
import frc.robot.auton.pathfollowing.motion.Translation;
import frc.robot.subsystems.Chassis;
import frc.robot.auton.path_planning.problem;

public class geometry{
    public static double dist(point p1, point p2){
        double x1 = p1.x;
        double x2 = p2.x;
        double dx = x2 - x1;
        double y1 = p1.y;
        double y2 = p2.y;
        double dy = y2 - y1;
        return Math.sqrt(dx * dx + dy * dy);
    }    

    public static point get_intersect(double x0, double y0, double t0, double x1, double y1, double t1){
        double x = ((y0 - y1 + x1 * Math.tan(t1) - x0 * Math.tan(t0))/(Math.tan(t1) - Math.tan(t0)));
        double y = (y0 + Math.tan(t0)*(x - x0));
        return new point(x, y); 
    }

    public static point solveSystem(double a, double b, double c, double d){
        double x = (d - b)/(a - c);
        double y = a * x + b;
        return new point(x, y);
    }

    public static problem genProblemFromVisionData(double A1, double A2, double l, RigidTransform curPose){
        double dx = Math.abs(l * Math.cos(geometry.deg2rad(A1)));
        double dy = Math.abs(l * Math.sin(geometry.deg2rad(A1)));
        double dtheta = geometry.deg2rad(Math.abs(A1)) + geometry.deg2rad(Math.abs(A2));
        RigidTransform rt = new RigidTransform( new Translation(dx, dy), Rotation.fromRadians(dtheta));
        RigidTransform finalPose = curPose.transformBy(rt);
        finalPose.transformBy(new RigidTransform(new Translation(50 * Math.sin(A2), 50 * Math.cos(A2)), Rotation.fromDegrees(0)));
        return new problem(curPose.getTranslation().x(), curPose.getTranslation().y(), curPose.getRotation().getRadians(), finalPose.getTranslation().x(), finalPose.getTranslation().y(), dtheta + curPose.getRotation().getRadians());
    }

    public static double deg2rad(double deg){
        return deg * Math.PI / 180;
    }
}