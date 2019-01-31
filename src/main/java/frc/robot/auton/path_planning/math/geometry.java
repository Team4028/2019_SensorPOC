package frc.robot.auton.path_planning.math;

import java.lang.Math;

import frc.robot.auton.path_planning.problem;
import frc.robot.auton.path_planning.math.point;
import frc.robot.auton.path_planning.math.linearHermiteSpline.pointSlope;
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

    public static double[] getArcFromPointSlopes(pointSlope ps1, pointSlope ps2){
        double x0 = ps1.pt.x;
        double y0 = ps1.pt.y;
        double x1 = ps2.pt.x;
        double y1 = ps1.pt.y;
        double m = ps2.slope;
        double a = -1 * m;
        double b = m * y1 + x1;
        double c = (y0 - y1)/(x1 - x0);
        double d = (y1 * y1 - y0 * y0 + x1 * x1 - x0 * x0)/(2 * x1 - 2 * x0);
        point locPoint = geometry.solveSystem(a, b, c, d);
        double h = locPoint.x;
        double k = locPoint.y;
        return new double[] {x0, y0, x1, y1, h, k};
    }

    public static double getRadiusFromPointSlopes(pointSlope ps1, pointSlope ps2){
        double[] arc = getArcFromPointSlopes(ps1, ps2);
        point p = new point(arc[0], arc[1]);
        point c = new point(arc[4], arc[5]);
        return geometry.dist(p, c);
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