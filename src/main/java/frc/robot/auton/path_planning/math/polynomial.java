package frc.robot.auton.path_planning.math;

import java.lang.Math;
import frc.robot.auton.path_planning.math.util;

public class polynomial{
    public double[] coeficientList;
    public int deg;

    public polynomial(double[] coefs){
        coeficientList = coefs;
        deg = coefs.length - 1;
    }

    public double evaluate(double x){
        double res = 0;
        for (int i = 0; i <= deg; i++){
            res += coeficientList[i] * Math.pow(x, i);
        }
        return res;
    }

    public polynomial reverse(){
        return new polynomial(util.reverse(coeficientList));
    }

    public polynomial derivative(){
        double[] derivCoef = new double[deg];
        for (int i = 1; i <= deg; i++){
            derivCoef[i - 1] = i * coeficientList[i];
        }
        return new polynomial(derivCoef);
    }

    public double evaluate_derivative(double x){
        return this.derivative().evaluate(x);
    }

    public polynomial scalar_multiply(double lambda){
        double[] res = new double[deg + 1];
        for (int i = 0; i <= deg; i++){
            res[i] = lambda * coeficientList[i];
        }
        return new polynomial(res);
    }

    public static polynomial add(polynomial p1, polynomial p2){
        double[] res;
        if (p1.deg == p2.deg){
            res = new double[p1.deg];
            for (int i = 0; i <= p1.deg; i++){
                res[i] = p1.coeficientList[i] + p2.coeficientList[i];
            }
        } else if (p1.deg > p2.deg){
            res = new double[p1.deg];
            for (int i = 0; i <= p2.deg; i++){
                res[i] = p1.coeficientList[i] + p2.coeficientList[i];
            }
            for (int j = p2.deg + 1; j <= p1.deg; j++){
                res[j] = p1.coeficientList[j];
            }
        } else {
            res = new double[p2.deg];
            for (int i = 0; i <= p1.deg; i++){
                res[i] = p1.coeficientList[i] + p2.coeficientList[i];
            }
            for (int j = p1.deg + 1; j <= p2.deg; j++){
                res[j] = p2.coeficientList[j];
            }            
        }
        return new polynomial(res);
    }

    public static polynomial mult(polynomial p1, polynomial p2){
        double[] res = new double[p1.deg + p2.deg + 1];
        for (int i = 0; i <= p1.deg; i++){
            for (int j = 0; j <= p2.deg; j++){
                res[i + j] += p1.coeficientList[i] * p2.coeficientList[j];
            }
        }
        return new polynomial(res);
    }

    public static polynomial Sigma(polynomial[] listOfPolynomials){
        if (listOfPolynomials.length == 0){
            return new polynomial(new double[] {0});
        } else if(listOfPolynomials.length == 1){
            return listOfPolynomials[0];
        } else {
            polynomial res = listOfPolynomials[0];
            for (int i = 1; i < listOfPolynomials.length; i++){
                res = polynomial.add(res, listOfPolynomials[i]);
            }
            return res;
        }
    }

    public static polynomial Pi(polynomial[] listOfPolynomials){
        if (listOfPolynomials.length == 0){
            return new polynomial(new double[] {0});
        } else if(listOfPolynomials.length == 1){
            return listOfPolynomials[0];
        } else {
            polynomial res = listOfPolynomials[0];
            for (int i = 1; i < listOfPolynomials.length; i++){
                res = polynomial.mult(res, listOfPolynomials[i]);
            }
            return res;
        }
    }

    public polynomial power(int p){
        if (p <= 0){
            return new polynomial(new double[] {1});
        } else {
            polynomial[] selves = new polynomial[p];
            for (int i = 0; i < p; i++){
                selves[i] = this;
            }
            return polynomial.Pi(selves);
        }
    }

    public static polynomial compose(polynomial f, polynomial g){
        polynomial[] addends = new polynomial[f.deg + 1];
        for (int i = 0; i <= f.deg; i++){
            addends[i] = g.power(i).scalar_multiply(f.coeficientList[i]);
        }
        return polynomial.Sigma(addends);
    }

    public static int var(double[] coefs){
        int tot = 0;
        for (int i = 0; i < coefs.length - 1; i++){
            if (Math.signum(coefs[i]) != Math.signum(coefs[i+1])){
                tot ++;
            }
        }
        return tot;
    }

    public polynomial divX(){
        double[] newArr = new double[deg];
        for (int i = 0; i <= deg - 1; i++){
            newArr[i] = coeficientList[i+1];
        }
        return new polynomial(newArr);
    }
}