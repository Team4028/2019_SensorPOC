package frc.robot.auton.path_planning.math;

import java.lang.Math;

public class interval{
    public double a;
    public double b;

    public interval(double left_val, double right_val){
        a = Math.min(left_val, right_val);
        b = Math.max(left_val, right_val);
    }

    public double get_length(){
        return b-a;
    }

    public double get_along(double t){
        return a + t * this.get_length();
    }

    public double get_midpoint(double t){
        return this.get_along(.5);
    }

    public static interval from_c_k_h(int c, int k, int h){
        return new interval( c / Math.pow(2, k), (c + h) / Math.pow(2, k));
    }

    public boolean is_inside(double val){
        return !(val > b || val < a);
    }
}