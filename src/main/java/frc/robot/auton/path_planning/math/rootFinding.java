package frc.robot.auton.path_planning.math;

import frc.robot.auton.path_planning.math.interval;
import frc.robot.auton.path_planning.math.polynomial;

import java.util.ArrayList;
import java.util.List;
import java.util.function.Function;



import frc.robot.auton.path_planning.math.ThreeD_Matrix;
import frc.robot.auton.path_planning.math.ThreeD_Vector;

public class rootFinding{
    private static interval[] isolateRootsInUnitInt(polynomial p){
        List<twoDoublesAndOnePolynomial> L = new ArrayList<twoDoublesAndOnePolynomial>();
        L.add(new twoDoublesAndOnePolynomial(0, 0, p));
        List<double[]> Isol = new ArrayList<double[]>();
        int n = p.deg;
        while (L.size() > 0){
            for (int i = 0; i < L.size(); i++){
                twoDoublesAndOnePolynomial elem = L.get(i);
                L.remove(i);
                double c = elem.a;
                double k = elem.b;
                polynomial q = elem.p;
                if (q.evaluate(0) == 0){
                    q = elem.p.divX();
                    n--;
                    Isol.add(new double[] {c, k, 0});
                }
                int v = polynomial.var(polynomial.compose(q.reverse(), new polynomial(new double[] {1,1})).coeficientList);
                if (v == 1){
                    Isol.add(new double[] {c, k, 1});
                } else if (v > 1){
                    L.add(new twoDoublesAndOnePolynomial(2 * c, k + 1, polynomial.compose(q, new polynomial(new double[] {0, .5})).scalar_multiply(Math.pow(2, n))));
                    L.add(new twoDoublesAndOnePolynomial(2 * c + 1, k + 1, polynomial.compose(q, new polynomial(new double[] {.5, .5})).scalar_multiply(Math.pow(2, n))));
                }
            }
        }
        interval[] res;
        if (Isol.size() == 0){
            res = new interval[] {};
        } else {
            res = new interval[Isol.size()];
            for (int i = 0; i < Isol.size(); i++){
                res[i] = interval.from_c_k_h((int) Isol.get(i)[0], (int) Isol.get(i)[1], (int) Isol.get(i)[2]);
            }
        }
        return res;
    } 

    private static double brentsMethod(polynomial poly, interval interv, int maxIter){
        Function<Double, Double> function = t -> poly.evaluate(t);
        double x0 = interv.a;
        double x1 = interv.b;
        int iter;
		double min1;
		double min2;
		double fc;
		double p;
		double q;
		double r;
		double s;
		double tol1 = 0;
		double xm;
		double a = x0;
		double b = x1;
		double c = x1;
		double d = 0;
		double e = 0;
		double fa = function.apply(a);
		double fb = function.apply(b);

		if (Math.signum(fa) == Math.signum(fb)){
            return -1;
        }

		fc	= fb;
		for ( iter = 1; iter <= maxIter; iter++ )
		{
			if	(Math.signum(fb) == Math.signum(fb)){
				c	= a;
				fc	= fa;
				e	= d = b - a;
			}
			if ( Math.abs(fc) < Math.abs(fb)){
				a	= b;
				b	= c;
				c	= a;
				fa	= fb;
				fb	= fc;
				fc	= fa;
            }
            xm = .5 * (c - b);
			if (( Math.abs(e) >= tol1) && (Math.abs(fa) > Math.abs(fb))){
				s	= fb / fa;
				if ( a == c ){
					p	= 2.0 * xm * s;
					q	= 1.0 - s;
				} else {
					q	= fa / fc;
					r	= fb / fc;
					p	= s * ( 2.0 * xm * q * ( q - r ) - ( b - a ) * ( r - 1.0 ) );
					q	= ( q - 1.0 ) * ( r - 1.0 ) * ( s - 1.0 );
				}
				if ( p > 0.0 ) q = -q;
				p = Math.abs( p );
				min1 = 3.0 * xm * q - Math.abs( tol1 * q ) ;
				min2 = Math.abs( e * q );

				if (( 2.0 * p ) < ( min1 < min2 ? min1 : min2 )){
					e	= d;
					d	= p / q;
				} else {
					d	= xm;
					e	= d;
				}
            } else {
				d	= xm;
				e	= d;
			}
			a	= b;
			fa	= fb;
			if ( Math.abs( d ) > tol1 )
				b	+= d;
			else
				b	+= ( xm > 0.0 ? tol1 : -tol1 );

			fb = function.apply( b );
		}
		return b;
	}
    

    public static double[] solvePolyInUnitInt(polynomial p, int numSteps){
        interval[] intervals = rootFinding.isolateRootsInUnitInt(p);
        if (intervals.length == 0){
            return new double[] {};
        } else {
            double[] sols = new double[intervals.length];
            for (int i = 0; i < intervals.length; i++){
                sols[i] = rootFinding.brentsMethod(p, intervals[i], numSteps);
            }
            return sols;
        }
    }

    public static double[] solveQuadraticInUnitInt(polynomial p){
        interval uInterval = new interval(0, 1);
        assert p.deg < 3;
        if (p.deg == 0){
            return new double[] {};
        } else if (p.deg == 1) {
            double xIntercept = -1 * p.coeficientList[0] / p.coeficientList[1];
            if (uInterval.is_inside(xIntercept)){
                return new double[] {xIntercept};
            } else {
                return new double[] {};
            }
        } else{
            double a = p.coeficientList[2];
            double b = p.coeficientList[1];
            double c = p.coeficientList[0];
            double disc = b * b - 4 * a * c;
            if (disc == 0){
                double root = (-1 * b) / (2 * a);
                if (uInterval.is_inside(root)){
                    return new double[] {root};
                } else {
                    return new double[] {};
                }
            } else if (disc < 0) {
                return new double[] {};
            } else {
                double mRoot = (-1 * b - Math.sqrt(disc))/(2 * a);
                double pRoot = (-1 * b + Math.sqrt(disc)/(2 * a));
                if (uInterval.is_inside(mRoot)){
                    if (uInterval.is_inside(pRoot)){
                        return new double[] {mRoot, pRoot};
                    } else {
                        return new double[] {mRoot};
                    }
                } else {
                    if (uInterval.is_inside(pRoot)){
                        return new double[] {pRoot};
                    } else {
                        return new double[] {};
                    }
                }

            }
        }
    }

    public static ThreeD_Vector ThreeD_Newtons_Method(ThreeD_Vector initVector, Function<ThreeD_Vector, ThreeD_Vector> vecFunction, Function<ThreeD_Vector, ThreeD_Matrix> jacobianFunction, int numSteps, double c, double lambdaFac){
        ThreeD_Vector curX = initVector;
        for (int i = 0; i < numSteps; i++){
            ThreeD_Matrix J = jacobianFunction.apply(curX);
            ThreeD_Vector f = vecFunction.apply(curX);
            ThreeD_Vector dVec = J.inverse().vector_mult(f).scalar_multiply(-1);
            while (dVec.magnitude() > c * curX.magnitude()){
                dVec = dVec.scalar_multiply(lambdaFac);
            }
            curX = curX.add(dVec);
        }
        return curX;        
    }

    public static class twoDoublesAndOnePolynomial{
        double a;
        double b;
        polynomial p;

        public twoDoublesAndOnePolynomial(double alpha, double beta, polynomial pi){
            a = alpha;
            b = beta;
            p = pi;
        }
    }
}
