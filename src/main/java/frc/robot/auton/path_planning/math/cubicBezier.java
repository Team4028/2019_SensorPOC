package frc.robot.auton.path_planning.math;

import frc.robot.auton.path_planning.math.point;
import frc.robot.auton.path_planning.math.lineSegment;
import frc.robot.auton.path_planning.math.linearHermiteSpline;
import frc.robot.auton.path_planning.math.util;
import frc.robot.auton.path_planning.math.quadraticBezier;
import java.lang.Math;
import frc.robot.auton.path_planning.math.polynomial;
import java.util.function.Function;
import frc.robot.auton.path_planning.math.rootFinding;
import frc.robot.auton.path_planning.math.ezOptimizer;

public class cubicBezier{

    private static final int numIterationsBrentsAlg = 50;
    point first_point;
    point second_point;
    point third_point;
    point fourth_point;

    public cubicBezier(point p1, point p2, point p3, point p4){
        first_point = p1;
        second_point = p2;
        third_point = p3;
        fourth_point = p4;
    }

    public point get_from_t(double t){
        point point_along_first_seg = new lineSegment(first_point, second_point).get_from_t(t);
        point point_along_second_seg = new lineSegment(second_point, third_point).get_from_t(t);
        point point_along_third_seg = new lineSegment(third_point, fourth_point).get_from_t(t);
        point point_val = new quadraticBezier(point_along_first_seg, point_along_second_seg, point_along_third_seg).get_from_t(t);
        return point_val;
    }

    public linearHermiteSpline approx_with_segs(int numSegs){
        lineSegment[] segments = new lineSegment[numSegs];
        double[] t_vals = util.linspace(0, 1, numSegs + 1);
        for (int i = 0; i < numSegs; i++){
            segments[i] = new lineSegment(this.get_from_t(t_vals[i]), this.get_from_t(t_vals[i+1]));
        }
        return new linearHermiteSpline(segments);
    }

    public double get_analytic_arc_len(int numTraps){
        double x0 = first_point.x;
        double x1 = second_point.x;
        double x2 = third_point.x;
        double x3 = fourth_point.x;
        double y0 = first_point.y;
        double y1 = second_point.y;
        double y2 = third_point.y;
        double y3 = fourth_point.y;
        double Ax = -3 * x0 + 9 * x1 - 9 * x2 + 3 * x3;
        double Bx = 6 * x0 - 12 * x1 + 6 * x2;
        double Cx = -3 * x0 + 3 * x1;
        double Ay = -3 * y0 + 9 * y1 - 9 * y2 + 3 * y3;
        double By = 6 * y0 - 12 * y1 + 6 * y2;
        double Cy = -3 * y0 + 3 * y1;
        double alpha = Ax * Ax + Ay * Ay;
        double beta = 2 * Ax * Bx + 2 * Ay * By;
        double gamma = 2 * Ax * Cx + Bx * Bx + 2 * Ay * Cy + By * By;
        double delta = 2 * Bx * Cx + 2 * By * Cy;
        double epsilon = Cx * Cx + Cy * Cy;
        polynomial veloPoly = new polynomial(new double[] {epsilon, delta, gamma, beta, alpha});
        Function<Double, Double> velo = t -> Math.sqrt(veloPoly.evaluate(t));
        return calculus.nRiemannTrapezoidIntegrate(velo, 0, 1, numTraps);
    }

    public double get_analytic_curvature(double t){
        double x0 = first_point.x;
        double x1 = second_point.x;
        double x2 = third_point.x;
        double x3 = fourth_point.x;
        double y0 = first_point.y;
        double y1 = second_point.y;
        double y2 = third_point.y;
        double y3 = fourth_point.y;
        double Ax = -3 * x0 + 9 * x1 - 9 * x2 + 3 * x3;
        double Bx = 6 * x0 - 12 * x1 + 6 * x2;
        double Cx = -3 * x0 + 3 * x1;
        double Ay = -3 * y0 + 9 * y1 - 9 * y2 + 3 * y3;
        double By = 6 * y0 - 12 * y1 + 6 * y2;
        double Cy = -3 * y0 + 3 * y1;
        double alpha = Ax * Ax + Ay * Ay;
        double beta = 2 * Ax * Bx + 2 * Ay * By;
        double gamma = 2 * Ax * Cx + Bx * Bx + 2 * Ay * Cy + By * By;
        double delta = 2 * Bx * Cx + 2 * By * Cy;
        double epsilon = Cx * Cx + Cy * Cy;
        polynomial veloPoly = new polynomial(new double[] {epsilon, delta, gamma, beta, alpha});
        double kappa = ((Ax * t * t + Bx * t + Cx) * (2 * Ay * t + By) - (2 * Ax *t + Bx) * (Ay * t * t + By * t + Cy))/Math.pow(veloPoly.evaluate(t), 1.5);
        return kappa;
    }

    public double get_analytic_init_curvature(){
        return this.get_analytic_curvature(0);  //It may be much better to actually do this explicitly, which is in the Py Source, but I think there's probably little difference
    }

    public double get_analytic_final_curvature(){
        return this.get_analytic_curvature(1); //Ditto to above
    }

    public double get_analytic_curvature_prime(double t){
        double x0 = first_point.x;
        double x1 = second_point.x;
        double x2 = third_point.x;
        double x3 = fourth_point.x;
        double y0 = first_point.y;
        double y1 = second_point.y;
        double y2 = third_point.y;
        double y3 = fourth_point.y;
        double Ax = -3 * x0 + 9 * x1 - 9 * x2 + 3 * x3;
        double Bx = 6 * x0 - 12 * x1 + 6 * x2;
        double Cx = -3 * x0 + 3 * x1;
        double Ay = -3 * y0 + 9 * y1 - 9 * y2 + 3 * y3;
        double By = 6 * y0 - 12 * y1 + 6 * y2;
        double Cy = -3 * y0 + 3 * y1;
        double alpha = Ax * Ax + Ay * Ay;
        double beta = 2 * Ax * Bx + 2 * Ay * By;
        double gamma = 2 * Ax * Cx + Bx * Bx + 2 * Ay * Cy + By * By;
        double delta = 2 * Bx * Cx + 2 * By * Cy;
        double epsilon = Cx * Cx + Cy * Cy;
        double constant_one = 2 * Bx * Ay * t - 2 * Ax * By * t + 2 * Cx * Ay  - 2 * Ax * Cy;
        double constant_two = alpha * t * t * t * t + beta * t * t * t + gamma * t * t + delta * t + epsilon;
        double constant_three = 4 * alpha * t * t * t + 3 * beta * t * t + 2 * gamma * t + delta;
        double constant_four = Bx * Ay * t * t + 2 * Cx * Ay * t + Cx * By - Ay * Bx * t * t - 2 * Ax * Cy * t - Bx * Cy;
        double composite_constant_one = constant_one * (Math.pow(constant_two , 1.5));
        double composite_constant_two = 1.5 * Math.sqrt(Math.abs(constant_two)) * constant_three * constant_four;
        double composite_constant_three = Math.pow(constant_two, 3);
        double d_kurvature_d_t = (composite_constant_one - composite_constant_two)/composite_constant_three;
        return d_kurvature_d_t;   
    }
    
    public double[] get_max_squared_curvature(){
        double x0 = first_point.x;
        double x1 = second_point.x;
        double x2 = third_point.x;
        double x3 = fourth_point.x;
        double y0 = first_point.y;
        double y1 = second_point.y;
        double y2 = third_point.y;
        double y3 = fourth_point.y;
        double Ax = -3 * x0 + 9 * x1 - 9 * x2 + 3 * x3;
        double Bx = 6 * x0 - 12 * x1 + 6 * x2;
        double Cx = -3 * x0 + 3 * x1;
        double Ay = -3 * y0 + 9 * y1 - 9 * y2 + 3 * y3;
        double By = 6 * y0 - 12 * y1 + 6 * y2;
        double Cy = -3 * y0 + 3 * y1;
        double alpha = Ax * Ax + Ay * Ay;
        double beta = 2 * Ax * Bx + 2 * Ay * By;
        double gamma = 2 * Ax * Cx + Bx * Bx + 2 * Ay * Cy + By * By;
        double delta = 2 * Bx * Cx + 2 * By * Cy;
        double epsilon = Cx * Cx + Cy * Cy;
        polynomial veloPoly = new polynomial(new double[] {epsilon, delta, gamma, beta, alpha});
        polynomial xVeloPoly = new polynomial(new double[] {Cx, Bx, Ax});
        polynomial yVeloPoly = new polynomial(new double[] {Cy, By, Ay});
        double[] nullPointsX = rootFinding.solveQuadraticInUnitInt(xVeloPoly);
        double[] nullPointsY = rootFinding.solveQuadraticInUnitInt(yVeloPoly);
        double[] nullPoints = util.intersect(nullPointsX, nullPointsY);
        if (nullPoints.length > 0){
            return new double[] {nullPoints[0], Double.POSITIVE_INFINITY};
        } else {
            polynomial poly_one = new polynomial(new double[] {2 * Cx * Ay - 2 * Ax * Cy, 2 * Bx * Ay - 2 * Ax * By});
            polynomial composite_poly_one = polynomial.mult(poly_one, veloPoly);
            polynomial poly_two = veloPoly.derivative();
            polynomial poly_three = new polynomial(new double[] {Cx * By - Bx * Cy, 2 * Cx * Ay- 2 * Ax * Cy, Bx * Ay - Ax * By});
            polynomial composite_poly_two = polynomial.mult(poly_two, poly_three).scalar_multiply(-1.5);
            polynomial derivKurvPoly = polynomial.add(composite_poly_one, composite_poly_two);
            double[] poss_vals = rootFinding.solvePolyInUnitInt(derivKurvPoly, numIterationsBrentsAlg);
            double[] testVals = new double[poss_vals.length + 2];
            testVals[0] = 0;
            testVals[1] = 1;
            for (int i = 0; i < poss_vals.length; i++){
                testVals[2 + i] = poss_vals[i];
            }
            double[] kappaSquaredVals = new double[testVals.length];
            for (int i = 0; i< testVals.length; i++){
                kappaSquaredVals[i] = Math.pow(this.get_analytic_curvature(testVals[i]), 2);
            }
            return util.getMaxPair(testVals, kappaSquaredVals);            
        }
    }

    public double lazyMaxKappaSquared(int num){
        Function<Double, Double> kappaSquared = t -> Math.pow(this.get_analytic_curvature(t), 2);
        return ezOptimizer.lazyOptimize(kappaSquared, 0, 1, num);
    }

    public void _print_control_points(){
        first_point.print();
        second_point.print();
        third_point.print();
        fourth_point.print();
    }
}
        