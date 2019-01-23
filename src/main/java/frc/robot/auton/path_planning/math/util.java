package frc.robot.auton.path_planning.math;

import java.util.ArrayList;
import java.util.stream.DoubleStream;

public class util{
    public static double[] reverse(double[] data) {
        int left = 0;
        int right = data.length - 1;
    
        while( left < right ) {
            double temp = data[left];
            data[left] = data[right];
            data[right] = temp;
            left++;
            right--;
        }

        return data;
    }

    public static double[] linspace(double start, double end, int num){
        double dx = (end - start)/(num - 1);
        double[] res = new double[num];
        for (int i = 0; i < num; i++){
            res[i] = start + i * dx;
        }
        return res;
    }

    public static boolean contains(double[] arrToCheck, double valToCheckFor){
        return DoubleStream.of(arrToCheck).anyMatch(x -> x == valToCheckFor);
    }

    public static double[] doubleListToArr(ArrayList<Double> arr){
        double[] res = new double[arr.size()];
        for (int i = 0; i < res.length; i++){
            res[i] = arr.get(i).doubleValue();
        }
        return res;
    }

    public static double[] intersect(double[] first, double[] second){
        ArrayList<Double> intersection = new ArrayList<Double>();
        for (int i = 0; i < first.length; i++){
            if (util.contains(second, first[i])){
                intersection.add(first[i]);
            }
        }
        return util.doubleListToArr(intersection);
    }

    public static double[] getMaxPair(double[] args, double[] results){
        int minInd = 0;
        double maxVal = Double.NEGATIVE_INFINITY;
        for (int i = 0; i < results.length; i++){
            if (results[i] > maxVal){
                maxVal = results[i];
                minInd = i;
            }
        }
        return new double[] {args[minInd], maxVal};
    }
}
