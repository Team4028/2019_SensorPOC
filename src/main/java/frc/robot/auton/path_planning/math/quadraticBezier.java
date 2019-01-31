package frc.robot.auton.path_planning.math;

import frc.robot.auton.path_planning.math.lineSegment;
import frc.robot.auton.path_planning.math.point;
import frc.robot.auton.path_planning.math.linearHermiteSpline;
import frc.robot.auton.path_planning.math.util;

public class quadraticBezier{
    point starting_point;
    point middle_point;
    point ending_point;

    public quadraticBezier(point p1, point p2, point p3){
        starting_point = p1;
        middle_point = p2;
        ending_point = p3;
    }

    public point get_from_t(double t){
        point point_along_first_seg = new lineSegment(starting_point, middle_point).get_from_t(t);
        point point_along_second_seg = new lineSegment(middle_point, ending_point).get_from_t(t);
        point point_val = new lineSegment(point_along_first_seg, point_along_second_seg).get_from_t(t);
        return point_val;
    }

    public linearHermiteSpline approx_with_segs(int numSegs){
        lineSegment[] segments = new lineSegment[numSegs];
        double[] t_vals = util.linspace(0, 1, numSegs);
        for (int i = 0; i < numSegs - 1; i++){
            segments[i] = new lineSegment(this.get_from_t(t_vals[i]), this.get_from_t(t_vals[i+1]));
        }
        return new linearHermiteSpline(segments);
    }
}