package frc.robot.auton.path_planning;

import frc.robot.auton.path_planning.math.geometry;
import frc.robot.auton.path_planning.math.point;

import java.util.ArrayList;
import java.util.List;
import java.util.function.Function;

import edu.wpi.first.wpilibj.Timer;
import frc.robot.auton.pathfollowing.PathBuilder;
import frc.robot.auton.pathfollowing.PathBuilder.Waypoint;
import frc.robot.auton.pathfollowing.control.Path;
import frc.robot.auton.pathfollowing.motion.RigidTransform;
import frc.robot.subsystems.Chassis;
import frc.robot.auton.path_planning.PathPlanner;
import frc.robot.Constants;
import frc.robot.auton.pathfollowing.PathBuilder;

public class problem{

    private static final int linearHermiteSplineLengthForArcLength = 100;
    private static final double maxKappaSquared = 1;
    private static final double epsilon = Constants.EPSILON_NEGATIVE_6;
    private static final int numStepsNewtonsMethod = 100;
    private static final double newtonsMethodC = 1;
    private static final double newtonsMethodLambdaFac = .5;
    private static final double cruiseVelo = 50;
    private static final double cruiseAccel = 50;
    private static final double cycleTime = .2;
    public static boolean pathPlanned = false;
    public static Path _path;
    public static double _theta;

    private static final int rGrid = 10;
    private static final int sGrid = 10;
    private static final int kapNum = 10;
    private static final int numLinSegs = 10;
    private static final int numSegmentsApproxForPathGen = 100;

    double Xi;
    double Yi; 
    double THETAi;
    double Xf;
    double Yf;
    double THETAf;

    public problem(double x0, double y0, double t0, double x1, double y1, double t1){
        Xi = x0;
        Yi = y0;
        THETAi = t0;
        Xf = x1;
        Yf = y1;
        THETAf = t1;
    }


    public void printStuff(){
        System.out.println("Xi: " + Xi);
        System.out.println("Yi: " + Yi);
        System.out.println("Theta i: " + THETAi);
        System.out.println("Xf: " + Xf);
        System.out.println("Yf: " + Yf);
        System.out.println("Theta f: " + THETAf);
    }

    public static void solveFromVisionData(double a1, double a2, double l, RigidTransform curPose){
        geometry.genProblemFromVisionData(a1, a2, l, curPose).pSolve();
    }

    public static void ezMoneySolveFromVisionData(double a1, double a2, double distance, RigidTransform curPose){
        System.out.println("Angle1:"+a1);
        System.out.println("Angle2:"+a2);
        System.out.println("Distance"+ distance);
        problem p = geometry.genProblemFromVisionData(a1, a2, distance, curPose);
        p.printStuff();
        p.pSolve();
    }

    public void pSolve(){
        point intPoint = geometry.get_intersect(Xi, Yi, THETAi, Xf, Yf, THETAf);
        boolean isViable = problem.isIntViable(Xi, Yi, Xf, Yf, intPoint, THETAf);
        double thetaTurn;
        List<Waypoint> sWaypoints = new ArrayList<Waypoint>();
        // if (isViable && geometry.dist(intPoint, new point(Xf,Yf))>20)
        // {
            sWaypoints.add(new Waypoint(Math.abs(Yi),-1 * Math.abs(Xi), 0,0));
            sWaypoints.add(new Waypoint(Math.abs(intPoint.y), -1 *Math.abs(intPoint.x), Math.min(geometry.dist(intPoint, new point(Xi,Yi)),geometry.dist(intPoint, new point(Xf,Yf))),cruiseVelo));
            sWaypoints.add(new Waypoint(Math.abs(Yf), -1 * Math.abs(Xf), 0, cruiseVelo));
            thetaTurn = 0;
        // }
        // else
        // {
        //     point newMiddlePoint = new point(Xf - 20 * Math.sin(THETAf), Yf - 20 * Math.cos(THETAf));
        //     thetaTurn = Math.abs(Math.atan2(newMiddlePoint.y - Yi, newMiddlePoint.x - Xi));
        //     sWaypoints.add(new Waypoint(Math.abs(Yi), -1 * Math.abs(Xi), 0,0));
        //     sWaypoints.add(new Waypoint(Math.abs(newMiddlePoint.y), -1 * Math.abs(newMiddlePoint.x), Math.min(geometry.dist(newMiddlePoint, new point(Xi,Yi)), 18), cruiseVelo));
        //     sWaypoints.add(new Waypoint(Math.abs(Yf), -1 * Math.abs(Xf),  0, cruiseVelo));
        // }
        System.out.println(sWaypoints);
        System.out.println("Target Angle: " + thetaTurn);
        _path = PathBuilder.buildPathFromWaypoints(sWaypoints);
        _theta = problem.rad2deg(thetaTurn);
    }


    public static double rad2deg(double rad){
        return rad * 180 / Math.PI;
    }
    public static double deg2rad(double deg){
        return deg*Math.PI /180 ;
    }


    public static boolean isIntViable(double x0, double y0, double x1, double y1, point intP, double thetaF){
        double m = Math.tan(thetaF + Math.PI/2);
        double b = y1 - m * x1;
        return isAbove(x0, y0, m , b) == isAbove(intP.x, intP.y, m , b);
    }

    public static boolean isAbove(double x0, double y0, double m, double  b){
        return y0 > m * x0 + b;        
    }

    public static Path getPath()
    {
        return _path;
    }
}
