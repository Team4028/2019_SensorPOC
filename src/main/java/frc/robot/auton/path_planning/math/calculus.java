package frc.robot.auton.path_planning.math;

import java.util.function.Function;
import java.lang.Math;

public class calculus{
    public static double nRiemannTrapezoidIntegrate(Function<Double, Double> funct, double lower, double upper, int numTraps){
        double area = 0;
        double dx = (upper - lower)/numTraps;
        for (int i = 0; i < numTraps - 1; i++){
            double leftVal = lower + i * dx;
            double rightVal = leftVal + dx;
            double leftHeight = funct.apply(leftVal);
            double rightHeight = funct.apply(rightVal);
            double averageHeight = (leftHeight + rightHeight)/2;
            double dArea = averageHeight * dx;
            area += dArea;
        }
        return area;
    }

    public static double nDerivative(Function<Double, Double> funct, double val, double epsilon){
        return (funct.apply(val + epsilon) - funct.apply(val))/epsilon;
    }

    public static double nSecondDerivative(Function<Double, Double> funct, double val, double epsilon){
        Function<Double, Double> deriv = t -> calculus.nDerivative(funct, t, epsilon);
        return calculus.nDerivative(deriv, val, epsilon);
    }

    public static double nCurvature(Function<Double, Double> xFunct, Function<Double, Double> yFunct, double t_val, double epsilon){
        double xPrime = calculus.nDerivative(xFunct, t_val, epsilon);
        double xPrimePrime = calculus.nSecondDerivative(xFunct, t_val, epsilon);
        double yPrime = calculus.nDerivative(yFunct, t_val, epsilon);
        double yPrimePrime = calculus.nSecondDerivative(yFunct, t_val, epsilon);
        double kappa = (xPrime * yPrimePrime - xPrimePrime * yPrime)/(Math.pow(xPrime * xPrime + yPrime * yPrime, 1.5));
        return kappa;
    }

    public static double nDerivCurvature(Function<Double, Double> xFunct, Function<Double, Double> yFunct, double t_val, double epsilon){
        Function<Double, Double> curv = t -> calculus.nCurvature(xFunct, yFunct, t, epsilon);
        return calculus.nDerivative(curv, t_val, epsilon);
    }
}