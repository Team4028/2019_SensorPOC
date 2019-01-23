package frc.robot.auton.path_planning.math;

import frc.robot.Constants;
import frc.robot.auton.path_planning.math.ThreeD_Vector;

public class ThreeD_Matrix{
    private double[][] _mat = new double[3][3];

    public ThreeD_Matrix(double[][] _array){
        _mat = _array;                
    }

    public double[][] asArray(){
        return _mat;
    }

    public static double[][] zeroArray(){
        return new double[][] {{0, 0, 0}, {0, 0, 0}, {0, 0, 0}};
    }

    public static ThreeD_Matrix eye(){
        return new ThreeD_Matrix(new double[][] {{1, 0, 0}, {0, 1, 0}, {0, 0, 1}});
    }

    public static double twoDDet(double a, double b, double c, double d){
        return a * d - b * c;
    }

    public ThreeD_Matrix scalar_multiply(double lambda){
        double[][] newArr = ThreeD_Matrix.zeroArray();
        for (int i = 0; i < 3; i++){
            for (int j = 0; j < 3; j++){
                newArr[i][j] = lambda * _mat[i][j];
            }
        }
        return new ThreeD_Matrix(newArr);
    }

    public double get(int i, int j){
        return _mat[i][j];
    }

    public ThreeD_Matrix transpose(){
        double[][] newArr = ThreeD_Matrix.zeroArray();
        for (int i = 0; i < 3; i++){
            for (int j = 0; j < 3; j++){
                newArr[i][j] = _mat[j][i];
            }
        }
        return new ThreeD_Matrix(newArr);
    }

    public ThreeD_Matrix mat_of_minors(){
        double[][] newArr = ThreeD_Matrix.zeroArray();
        newArr[0][0] = ThreeD_Matrix.twoDDet(_mat[1][1], _mat[1][2], _mat[2][1], _mat[2][2]);
        newArr[0][1] = ThreeD_Matrix.twoDDet(_mat[1][0], _mat[1][2], _mat[2][0], _mat[2][2]);
        newArr[0][2] = ThreeD_Matrix.twoDDet(_mat[1][0], _mat[1][1], _mat[2][0], _mat[2][1]);
        newArr[1][0] = ThreeD_Matrix.twoDDet(_mat[0][1], _mat[0][2], _mat[2][1], _mat[2][2]);
        newArr[1][1] = ThreeD_Matrix.twoDDet(_mat[0][0], _mat[0][2], _mat[2][0], _mat[2][2]);
        newArr[1][2] = ThreeD_Matrix.twoDDet(_mat[0][0], _mat[0][1], _mat[1][0], _mat[2][1]);
        newArr[2][0] = ThreeD_Matrix.twoDDet(_mat[0][1], _mat[0][2], _mat[1][1], _mat[1][2]);
        newArr[2][1] = ThreeD_Matrix.twoDDet(_mat[0][0], _mat[0][2], _mat[1][0], _mat[1][2]);
        newArr[2][2] = ThreeD_Matrix.twoDDet(_mat[0][0], _mat[0][1], _mat[1][0], _mat[1][1]);
        return new ThreeD_Matrix(newArr);
    }

    public ThreeD_Matrix cofactor_matrix(){
        double[][] newArr = this.mat_of_minors().asArray();
        newArr[0][1] *= -1;
        newArr[1][0] *= -1;
        newArr[1][2] *= -1;
        newArr[2][1] *= -1;
        return new ThreeD_Matrix(newArr);
    }

    public ThreeD_Matrix adjoint_matrix(){
        return this.cofactor_matrix().transpose();
    }

    public double det(){
        double first_addend = this.get(0,0) * ThreeD_Matrix.twoDDet(this.get(1, 1), this.get(1, 2), this.get(2, 1), this.get(2, 2));
        double second_addend = this.get(0, 1) * ThreeD_Matrix.twoDDet(this.get(1, 0), this.get(1, 2), this.get(2, 0), this.get(2, 2));
        double third_addend = this.get(0, 2) * ThreeD_Matrix.twoDDet(this.get(1, 0), this.get(1, 1), this.get(2, 0), this.get(2, 1));
        return first_addend - second_addend + third_addend;
    }

    public ThreeD_Matrix inverse(){
        double d = this.det();
        // This is used primarily for Newton's Method. So, if the jacobian is singular, we move just very slightly via the identity matrix to determine where to go next
        if (d == 0){
            return ThreeD_Matrix.eye().scalar_multiply(Constants.EPSILON_NEGATIVE_6);
        } else {
            return this.adjoint_matrix().scalar_multiply(1/d);
        }
    }

    public ThreeD_Vector vector_mult(ThreeD_Vector vec){
        double top = this.get(0, 0) * vec.get(0) + this.get(0, 1) * vec.get(1) + this.get(0, 2) * vec.get(2);
        double middle = this.get(1, 0) * vec.get(0) + this.get(1, 1) * vec.get(1) + this.get(1, 2) * vec.get(2);
        double bottom = this.get(2, 0) * vec.get(0) + this.get(2, 1) * vec.get(1) + this.get(2, 2) * vec.get(2);
        double[] newArr = new double[] {top, middle, bottom};
        return new ThreeD_Vector(newArr);
    }
}