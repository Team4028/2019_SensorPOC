package frc.robot.auton.path_planning;

import frc.robot.auton.path_planning.math.cubicBezier;
import frc.robot.auton.path_planning.math.geometry;
import frc.robot.auton.path_planning.math.point;
import frc.robot.auton.path_planning.math.lineSegment;
import frc.robot.auton.path_planning.math.linearHermiteSpline;
import java.util.function.Function;

import edu.wpi.first.wpilibj.Timer;
import frc.robot.auton.pathfollowing.control.Path;
import frc.robot.auton.pathfollowing.motion.RigidTransform;
import frc.robot.auton.path_planning.PathPlanner;
import frc.robot.Constants;
import frc.robot.auton.path_planning.math.ThreeD_Matrix;
import frc.robot.auton.path_planning.math.ThreeD_Vector;
import frc.robot.auton.path_planning.math.calculus;
import frc.robot.auton.path_planning.math.rootFinding;
import frc.robot.auton.path_planning.math.ezOptimizer;

public class problem{

    private static final int linearHermiteSplineLengthForArcLength = 100;
    private static final double maxKappaSquared = 0.000074517;
    private static final double epsilon = Constants.EPSILON_NEGATIVE_6;
    private static final int numStepsNewtonsMethod = 100;
    private static final double newtonsMethodC = 1;
    private static final double newtonsMethodLambdaFac = .5;
    private static final double cruiseVelo = 40;
    private static final double cruiseAccel = Constants.PATH_DEFAULT_ACCEL;
    private static final double cycleTime = .2;
    public static boolean pathPlanned = false;
    public static Path _path;

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

    public cubicBezier genBezier(double r, double s) {
        point first_control_point = new lineSegment(new point(Xi, Yi), geometry.get_intersect(Xi, Yi, THETAi, Xf, Yf, THETAf)).get_length_along(r);
        point second_control_point = new lineSegment(new point(Xf, Yf), geometry.get_intersect(Xi, Yi, THETAi, Xf, Yf, THETAf)).get_length_along(s);
        return new cubicBezier(new point(Xi, Yi), first_control_point, second_control_point, new point(Xf, Yf));
    }

    public double l(double r, double s){
        return genBezier(r, s).approx_with_segs(linearHermiteSplineLengthForArcLength).get_arc_len();
    }

    public double psi(double r, double s){
        return this.genBezier(r, s).get_max_squared_curvature()[1] - 1/maxKappaSquared;
    }

    public double l_r(double r, double s){
        Function<Double, Double> _locFunction = t -> this.l(t, s);
        return calculus.nDerivative(_locFunction, r, epsilon);
    }

    public double l_s(double r, double s){
        Function<Double, Double> _locFunction = t -> this.l(r, t);
        return calculus.nDerivative(_locFunction, s, epsilon);
    }

    public double l_rr(double r, double s){
        Function<Double, Double> _locFunction = t -> this.l_r(t, s);
        return calculus.nDerivative(_locFunction, r, epsilon);
    }

    public double l_rs(double r, double s){
        Function<Double, Double> _locFunction = t -> this.l_r(r, t);
        return calculus.nDerivative(_locFunction, s, epsilon);
    }

    public double l_sr(double r, double s){
        Function<Double, Double> _locFunction = t -> this.l_s(t, s);
        return calculus.nDerivative(_locFunction, r, epsilon);
    }

    public double l_ss(double r, double s){
        Function<Double, Double> _locFunction = t -> this.l_s(r, t);
        return calculus.nDerivative(_locFunction, s, epsilon);
    }

    public double psi_r(double r, double s){
        Function<Double, Double> _locFunction = t -> this.psi(t, s);
        return calculus.nDerivative(_locFunction, r, epsilon);
    }

    public double psi_s(double r, double s){
        Function<Double, Double> _locFunction = t -> this.psi(r, t);
        return calculus.nDerivative(_locFunction, s, epsilon);
    }

    public double psi_rr(double r, double s){
        Function<Double, Double> _locFunction = t -> this.psi_r(t, s);
        return calculus.nDerivative(_locFunction, r, epsilon);
    }

    public double psi_rs(double r, double s){
        Function<Double, Double> _locFunction = t -> this.psi_r(r, t);
        return calculus.nDerivative(_locFunction, s, epsilon);
    }

    public double psi_sr(double r, double s){
        Function<Double, Double> _locFunction = t -> this.psi_s(t, s);
        return calculus.nDerivative(_locFunction, r, epsilon);
    }

    public double psi_ss(double r, double s){
        Function<Double, Double> _locFunction = t -> this.psi_s(r, t);
        return calculus.nDerivative(_locFunction, s, epsilon);
    }
    
    public ThreeD_Vector get_candidate(ThreeD_Vector startVec){
        Function<ThreeD_Vector, ThreeD_Vector> vecFunct = vec -> new ThreeD_Vector(new double[] {vec.get(2) * this.psi_r(vec.get(0), vec.get(1)) + this.l_r(vec.get(0),vec.get(1)), vec.get(2) * this.psi_s(vec.get(0), vec.get(1)) + this.l_s(vec.get(0), vec.get(1)), vec.get(2) * this.psi(vec.get(0),vec.get(1))});
        Function<ThreeD_Vector, ThreeD_Matrix> matFunct = vec -> new ThreeD_Matrix(new double[][] {{vec.get(2) * this.psi_rr(vec.get(0), vec.get(1)) + this.l_rr(vec.get(0), vec.get(1)), vec.get(2) * this.psi_rs(vec.get(0), vec.get(1)) + this.l_rs(vec.get(0), vec.get(1)), this.psi_r(vec.get(0), vec.get(1))}, {vec.get(2) * this.psi_sr(vec.get(0), vec.get(1)) + this.l_sr(vec.get(0), vec.get(1)), vec.get(2) * this.psi_ss(vec.get(0), vec.get(1)) + this.l_ss(vec.get(0), vec.get(1)), this.psi_s(vec.get(0), vec.get(1))}, {vec.get(2) * this.psi_r(vec.get(0), vec.get(1)), vec.get(2) * this.psi_s(vec.get(0), vec.get(1)), this.psi(vec.get(0), vec.get(1))}});
        return rootFinding.ThreeD_Newtons_Method(startVec, vecFunct, matFunct, numStepsNewtonsMethod, newtonsMethodC, newtonsMethodLambdaFac);
    }

    public ThreeD_Vector initCandidate(){
        if (Math.abs(Math.tan(THETAi)) != Math.abs(Math.tan(THETAf))){
            point intPoint = geometry.get_intersect(Xi, Yi, THETAi, Xf, Yf, THETAf);
            double r_init = .5 * geometry.dist(new point(Xi, Yi), intPoint);
            double s_init = .5 * geometry.dist(intPoint, new point(Xf, Yf));
            return new ThreeD_Vector(new double[] {r_init, s_init, 0});
        } else {
            double r_init = geometry.dist(new point(Xi, Yi), new point(Xf, Yf));
            return new ThreeD_Vector(new double[] {r_init, r_init, 0});
        }
    }

    public Path solve(){
        ThreeD_Vector vec = this.get_candidate(this.initCandidate());
        double r = vec.get(0);
        double s = vec.get(1);
        cubicBezier bezSol = this.genBezier(r, s);
        bezSol._print_control_points();
        linearHermiteSpline lhSpline = bezSol.approx_with_segs(numSegmentsApproxForPathGen);
        Path sol = PathPlanner.planPath(PathPlanner.genWaypointsFromSpline(lhSpline, cruiseVelo, cruiseAccel, cycleTime));
        return sol;
    }

    private double[] getMinMaxs(){
        if (Math.abs(Math.tan(THETAi)) != Math.abs(Math.tan(THETAf))){
            point intPoint = geometry.get_intersect(Xi, Yi, THETAi, Xf, Yf, THETAf);
            double d1 = geometry.dist(new point(Xi, Yi), intPoint);
            double d2 = geometry.dist(intPoint, new point(Xf, Yf));
            return new double[] {d1 * -1.5, d1 * -.1,  .3 * d2 , 1.5 * d2};
        } else {
            double d = geometry.dist(new point(Xi, Yi), new point(Xf, Yf));
            return new double[] {d * .06,  d * 1.25,  d * -1.25, d * -.3};
        }
    }

    public Path ezSolve(){
        double[] minMaxs = this.getMinMaxs();
        double t0 = Timer.getFPGATimestamp();
        double[] params = ezOptimizer.optimizeRS(this, minMaxs[0], minMaxs[1], minMaxs[2], minMaxs[3], maxKappaSquared, rGrid, sGrid, kapNum, numLinSegs);
        double dt0 = Timer.getFPGATimestamp() - t0;
        double t1 = Timer.getFPGATimestamp();
        cubicBezier bezSol = this.genBezier(params[0], params[1]);
        double dt1 = Timer.getFPGATimestamp() - t1;
        bezSol._print_control_points();
        double t2 = Timer.getFPGATimestamp();
        linearHermiteSpline lhSpline = bezSol.approx_with_segs(numSegmentsApproxForPathGen);
        double dt2 = Timer.getFPGATimestamp() - t2;
        double t3 = Timer.getFPGATimestamp();
        Path sol = PathPlanner.planPath(PathPlanner.genWaypointsFromSpline(lhSpline, cruiseVelo, cruiseAccel, cycleTime));
        double dt3 = Timer.getFPGATimestamp() - t3;
        System.out.println("RS calculation time [s]:  " + dt0);
        System.out.println("Bezier generation time [s]:  " + dt1);
        System.out.println("Linear Hermite Spline Generation Time [s]:  " + dt2);
        System.out.println("Path Generation Time [s]:  " + dt3);
        return sol;
    }

    public static Path solveFromVisionData(double a1, double a2, double l, RigidTransform curPose){
        return geometry.genProblemFromVisionData(a1, a2, l, curPose).solve();
    }

    public static Path ezMoneySolveFromVisionData(double a1, double a2, double l, RigidTransform curPose){
        // problem p = geometry.genProblemFromVisionData(a1, a2, l, curPose);
        problem p = new problem(0, 0, Math.PI/3, 150, 350, Math.PI/2);
        System.out.println(p.Xi);
        return p.ezSolve();
    }

    public static Path getPath()
    {
        return _path;
    }
}
