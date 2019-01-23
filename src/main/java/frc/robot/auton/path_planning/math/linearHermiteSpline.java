package frc.robot.auton.path_planning.math;

import frc.robot.auton.path_planning.math.lineSegment;

public class linearHermiteSpline{
    private lineSegment[] _segs;
    public int _size;

    public linearHermiteSpline(lineSegment[] segments){
        _segs = segments;
        _size = segments.length;
    }

    public double get_arc_len(){
        double tot_len = 0;
        for (int ind = 0; ind < _size; ind ++){
            tot_len += _segs[ind].arc_len();
        }
        return tot_len;
    }

    public point getFromLength(double len){
        double curLen = 0;
        int ind = -1;
        while (curLen <= len){
            ind++;
            curLen += _segs[ind].arc_len();
        }
        double lenDiff = len - curLen + _segs[ind].arc_len();
        return _segs[ind].get_length_along(lenDiff);
    }

    public pointSlope pointSlopeFromLength(double len){
        double curLen = 0;
        int ind = -1;
        while (curLen <= len){
            ind++;
            curLen += _segs[ind].arc_len();
        }
        double lenDiff = len - curLen + _segs[ind].arc_len();
        point ansPoint =  _segs[ind].get_length_along(lenDiff);
        double m = _segs[ind].slope();
        return new pointSlope(ansPoint, m);
    }

    public class pointSlope{
        public point pt;
        public double slope;

        private pointSlope(point p, double m){
            pt = p;
            slope = m;
        }
    }

    public double approxT(double arcLen){
        double curLen = 0;
        int ind = -1;
        while (curLen <= arcLen){
            ind++;
            curLen += _segs[ind].arc_len();
        }
        double lenDiff = arcLen - curLen + _segs[ind].arc_len();
        return (ind + (lenDiff/(_segs[ind].arc_len())))/(_size);
    }
}