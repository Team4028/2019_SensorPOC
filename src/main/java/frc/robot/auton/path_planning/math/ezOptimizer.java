package frc.robot.auton.path_planning.math;

import frc.robot.auton.path_planning.math.util;
import frc.robot.auton.path_planning.math.cubicBezier;

import java.util.function.Function;

import frc.robot.auton.path_planning.problem;

public class ezOptimizer{

    public static double[] optimizeRS(problem p, double minR, double maxR, double minS, double maxS, double maxKappaSquared, int rGrid, int sGrid, int kapNum, int numLinSegs){
        double[] rVals = util.linspace(minR, maxR, rGrid + 1);
        double[] sVals = util.linspace(minS, maxS, sGrid + 1);
        double[] bestPairSoFar = new double[] {Double.POSITIVE_INFINITY, Double.POSITIVE_INFINITY};
        double bestLenSoFar = Double.POSITIVE_INFINITY;
        for (int i = 0; i < rGrid + 1; i++){
            for (int j = 0; j < sGrid + 1; j++){
                cubicBezier bez = p.genBezier(rVals[i], sVals[j]);
                if (bez.lazyMaxKappaSquared(kapNum) < maxKappaSquared){
                    double l = bez.approx_with_segs(numLinSegs).get_arc_len();
                    if ( l < bestLenSoFar){
                        bestPairSoFar = new double[] {rVals[i], sVals[j]};
                        bestLenSoFar = l;
                    }
                }
            }
        }
        return bestPairSoFar;
    }

    public static double lazyOptimize(Function<Double, Double> f, double a, double b, int n){
        double bestSoFar = Double.NEGATIVE_INFINITY;
        double[] testVals = util.linspace(0, 1, n + 1);
        for (int i = 0; i < n + 1; i ++){
            double val = f.apply(testVals[i]);
            if (val > bestSoFar){
                bestSoFar = val;
            }
        }
        return bestSoFar;
    }
}