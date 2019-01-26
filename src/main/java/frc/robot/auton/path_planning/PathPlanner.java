package frc.robot.auton.path_planning;

import frc.robot.auton.path_planning.math.geometry;
import frc.robot.auton.path_planning.math.linearHermiteSpline;
import frc.robot.auton.pathfollowing.control.Path;
import frc.robot.auton.pathfollowing.PathBuilder;
import frc.robot.auton.pathfollowing.PathBuilder.Waypoint;
import frc.robot.auton.path_planning.math.linearHermiteSpline.pointSlope;
import java.util.ArrayList;
import java.util.List;
import java.util.function.Function;
import java.lang.Math;

public class PathPlanner {

    public static List<Waypoint>  genWaypointsFromSpline(linearHermiteSpline spline, double cruiseVelo, double accel, double cycleTime){
        List<oneDimMotionPrimitive> lengths = PathPlanner.getLengthsAndVelos(spline.get_arc_len(), cruiseVelo, accel, cycleTime);
        List<Waypoint> waypts = new ArrayList<Waypoint>();
        List<pointSlope> pointSlopes = new ArrayList<pointSlope>();
        List<Double> radii = new ArrayList<Double>();
        List<Double> velos = new ArrayList<Double>();
        for (int i = 0; i < lengths.size(); i++){
            velos.add(lengths.get(i).velo);
        }
        for (int i = 0; i < lengths.size(); i++){
            pointSlopes.add(spline.pointSlopeFromLength(lengths.get(i).len));            
        }
        for (int i = 0; i < lengths.size() - 1; i++){
            radii.add(geometry.getRadiusFromPointSlopes(pointSlopes.get(i), pointSlopes.get(i+1)));            
        }
        radii.add(0.);
        for (int i = 0; i < lengths.size(); i++){
            System.out.println("X: " + pointSlopes.get(i).pt.x);
            System.out.println("Y: " + pointSlopes.get(i).pt.y);
            System.out.println("R: " + radii.get(i));
            System.out.println("V: " + velos.get(i));
            waypts.add(new Waypoint(pointSlopes.get(i).pt.x, pointSlopes.get(i).pt.y, radii.get(i), velos.get(i)));
        }
        return waypts;         
    }

    public static List<oneDimMotionPrimitive> getLengthsAndVelos(double len, double velo, double accel, double cycleTime){
        List<oneDimMotionPrimitive> lensAndVelos = new ArrayList<oneDimMotionPrimitive>();
        lensAndVelos.add(new oneDimMotionPrimitive(0, 0));
        double totRampLen = velo*velo/accel;
        if (len < totRampLen){
            double realVelo = Math.sqrt(len * accel);
            double completionTime = 2 * realVelo / accel;
            int numCycles = (int) Math.ceil(completionTime / cycleTime);
            double criticalTime = completionTime / 2;
            double lenSoFar = 0.;
            double timeSoFar = 0.;
            Function<Double, Double> firstFunction = t -> accel * t; 
            Function<Double, Double> secondFunction = t -> realVelo - accel*(t - criticalTime);
            Function<Double, Double> getVelo = t -> t < criticalTime ? firstFunction.apply(t) : secondFunction.apply(t);
            for (int i = 0; i < numCycles; i++){              
                double initVelo = getVelo.apply(timeSoFar);
                double finalVelo = getVelo.apply(timeSoFar + cycleTime);
                timeSoFar += cycleTime;
                double avgVelo = (initVelo + finalVelo)/2;
                double dl = avgVelo * cycleTime;
                lenSoFar += dl;
                lensAndVelos.add(new oneDimMotionPrimitive(lenSoFar, finalVelo));
            }    
        } else {
            double constantVeloLen = len - totRampLen;
            double completionTime = (2 * velo / accel) + (constantVeloLen / velo);
            int numCycles = (int) Math.ceil(completionTime / cycleTime);
            double criticalTime1 = velo / accel;
            double criticalTime2 = completionTime - (velo / accel);
            double lenSoFar = 0.;
            double timeSoFar = 0.;
            Function<Double, Double> firstFunction = t -> accel * t; 
            Function<Double, Double> middleFunction = t -> velo;
            Function<Double, Double> thirdFunction = t -> velo - accel*(t - criticalTime2);
            for (int i = 0; i < numCycles; i++){              
                double initVelo = PathPlanner.getQuaternaryOperation(timeSoFar, criticalTime1, criticalTime2, firstFunction, middleFunction, thirdFunction);
                double finalVelo = PathPlanner.getQuaternaryOperation(timeSoFar + cycleTime, criticalTime1, criticalTime2, firstFunction, middleFunction, thirdFunction);
                timeSoFar += cycleTime;
                double avgVelo = (initVelo + finalVelo)/2;
                double dl = avgVelo * cycleTime;
                lenSoFar += dl;
                lensAndVelos.add(new oneDimMotionPrimitive(lenSoFar, finalVelo));
            }
        }
        return lensAndVelos;
    }

    public static double getQuaternaryOperation(double time, double t1, double t2, Function<Double, Double> f1, Function<Double, Double> f2, Function<Double, Double> f3){
        if (time < t1){
            return f1.apply(time);
        } else if (time > t2){
            return f3.apply(time);
        } else {
            return f2.apply(time);
        }
    }

    public static Path planPath(List<Waypoint> w){
        return PathBuilder.buildPathFromWaypoints(w);                
    }

    public static class oneDimMotionPrimitive{
        double len;
        double velo;

        public oneDimMotionPrimitive(double l, double v){
            len = l;
            velo = v;
        }
    }

    
}