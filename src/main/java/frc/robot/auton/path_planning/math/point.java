package frc.robot.auton.path_planning.math;

public class point{

    public double x;
    public double y;

    public point(double x_val, double y_val){
        x = x_val;
        y = y_val;
    }

    public void print(){
        String _printStr = "[" + Double.toString(x) + ", " + Double.toString(y) + "]";
        System.out.println(_printStr);
    }
}