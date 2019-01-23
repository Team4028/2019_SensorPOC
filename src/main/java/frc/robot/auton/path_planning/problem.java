package frc.robot.auton.path_planning;

import frc.robot.auton.path_planning.math.cubicBezier;
import frc.robot.auton.path_planning.math.geometry;
import frc.robot.auton.path_planning.math.point;
import frc.robot.auton.path_planning.math.lineSegment;
import frc.robot.auton.path_planning.math.linearHermiteSpline;
import java.util.function.Function;
import frc.robot.auton.pathfollowing.control.Path;
import frc.robot.auton.pathfollowing.motion.RigidTransform;
import frc.robot.auton.path_planning.PathPlanner;
import frc.robot.Constants;
import frc.robot.auton.path_planning.math.ThreeD_Matrix;
import frc.robot.auton.path_planning.math.ThreeD_Vector;
import frc.robot.auton.path_planning.math.calculus;
import frc.robot.auton.path_planning.math.rootFinding;

public class problem{

    private static final int linearHermiteSplineLengthForArcLength = 100;
    private static final double maxKappaSquared = 1/256;
    private static final double epsilon = Constants.EPSILON_NEGATIVE_6;
    private static final int numStepsNewtonsMethod = 100;
    private static final double newtonsMethodC = 1;
    private static final double newtonsMethodLambdaFac = .5;
    private static final int numSegmentsApproxForPathGen = 10000;
    private static final double cruiseVelo = 40;
    private static final double cruiseAccel = Constants.PATH_DEFAULT_ACCEL;
    private static final double cycleTime = .2;
    public static boolean pathPlanned = false;
    public static Path _path;

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
        linearHermiteSpline lhSpline = bezSol.approx_with_segs(numSegmentsApproxForPathGen);
        Path sol = PathPlanner.planPath(PathPlanner.genWaypointsFromSpline(lhSpline, cruiseVelo, cruiseAccel, cycleTime));
        return sol;
    }

    public static Path solveFromVisionData(double a1, double a2, double l, RigidTransform curPose){
        return geometry.genProblemFromVisionData(a1, a2, l, curPose).solve();
    }
}
