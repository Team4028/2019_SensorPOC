package frc.robot.auton.path_planning.math;

import frc.robot.auton.path_planning.math.geometry;
import frc.robot.auton.path_planning.math.point;
import java.lang.Math;

public class lineSegment{
    public point starting_point;
    public point ending_point;

    public lineSegment(point p1, point p2){
        starting_point = p1;
        ending_point = p2;
    }
    
    public point get_from_t(double t){
        double x_val = starting_point.x + t * (ending_point.x - starting_point.x);
        double y_val = starting_point.y + t * (ending_point.y - starting_point.y);
        return new point(x_val, y_val);
    }

    public double arc_len(){
        return geometry.dist(starting_point, ending_point);
    }

    public point get_length_along(double l){
        double theta = Math.atan2(ending_point.y - starting_point.y, ending_point.x - starting_point.x);
        return new point(starting_point.x +  l * Math.cos(theta), starting_point.y + l * Math.sin(theta));
    }

    public double slope(){
        return (ending_point.y - starting_point.y)/(ending_point.x - starting_point.x);
    }
}