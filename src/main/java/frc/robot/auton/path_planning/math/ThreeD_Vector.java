package frc.robot.auton.path_planning.math;

import java.lang.Math;

public class ThreeD_Vector{
    private double[] _vec = new double[3];

    public ThreeD_Vector(double[] array){
        _vec = array;
    }

    public double get(int ind){
        return _vec[ind];
    }

    public double magnitude(){
        return Math.sqrt(Math.pow(_vec[0], 2) + Math.pow(_vec[1], 2) + Math.pow(_vec[2], 2));
    }

    public ThreeD_Vector scalar_multiply(double lambda){
        return new ThreeD_Vector(new double[] {_vec[0] * lambda, _vec[1] * lambda, _vec[2] * lambda});
    }

    public ThreeD_Vector add(ThreeD_Vector other){
        return new ThreeD_Vector(new double[] {this.get(0) + other.get(0), this.get(1) + other.get(1), this.get(2)+other.get(2)});
    }
}