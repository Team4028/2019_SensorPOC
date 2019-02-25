package frc.robot.sensors.visionLLInterpretation;

public class FourByFourMatrix {

    private double[][] array;

    public FourByFourMatrix(double[][] arr){
        array = arr;            
    }

    public double get(int i, int j){
        return array[i][j];
    }

    public void set(int i, int j, double val){
        array[i][j] = val;
    }

    public static FourByFourMatrix identMatrix(){
        return new FourByFourMatrix(new double[][] {{1, 0, 0, 0},{0, 1, 0, 0},{0, 0, 1, 0},{0, 0, 0, 1}});
    }

    public static FourByFourMatrix zeros(){
        return new FourByFourMatrix(new double[][] {{0, 0, 0, 0},{0, 0, 0, 0},{0, 0, 0, 0},{0, 0, 0, 0}});
    }

    public static FourByFourMatrix zAxisRotationMatrix(double thetaZ){
        return new FourByFourMatrix(new double[][] {{Math.cos(thetaZ), -1 * Math.sin(thetaZ), 0, 0}, {Math.sin(thetaZ), Math.cos(thetaZ), 0, 0}, {0, 0, 1, 0}, {0, 0, 0, 1}});
    }

    public static FourByFourMatrix yAxisRotationMatrix(double thetaY){
        return new FourByFourMatrix(new double[][] {{Math.cos(thetaY), 0, -1 * Math.sin(thetaY), 0}, {0, 1, 0, 0}, {Math.sin(thetaY), 0, Math.cos(thetaY), 0}, {0, 0, 0, 1}});
    }

    public static FourByFourMatrix xAxisRotationMatrix(double thetaX){
        return new FourByFourMatrix(new double[][] {{1, 0, 0, 0}, {0, Math.cos(thetaX), -1 * Math.sin(thetaX), 0}, {0, Math.sin(thetaX), Math.cos(thetaX), 0}, {0, 0, 0, 1}});
    }

    public static FourByFourMatrix translationTransformationMatrix(double x, double y, double z){
        return new FourByFourMatrix(new double[][] {{1, 0, 0, x}, {0, 1, 0, y}, {0, 0, 1, z}, {0, 0, 0, 1}});
    }

    public FourByFourMatrix applyLinearTransformation(FourByFourMatrix other){
        FourByFourMatrix ans = FourByFourMatrix.zeros();
        for (int i = 0; i < 4; i++){
            for (int j = 0; j < 4; j++){
                ans.set(i, j, FourByFourMatrix.getMultElem(this, other, i, j));
            }
        }
        return ans;
    }

    private static double getMultElem(FourByFourMatrix mat1, FourByFourMatrix mat2, int i, int j){
        double ans = 0;
        for (int ind = 0; ind < 4; ind++){
            ans += mat2.get(ind, i) * mat1.get(j, ind);
        }
        return ans;
    }

    public FourByFourMatrix transpose(){
        double[][] transposeArr = new double[4][4];
        for (int i = 0; i < 4; i++){
            for (int j = 0; j < 4; j++){
                transposeArr[i][j] = this.get(j, i);
            }
        }
        return new FourByFourMatrix(transposeArr);
    }

    public FourByFourMatrix invertAsIsometry(){
        double R11 = this.get(0, 0);
        double R12 = this.get(0, 1);
        double R13 = this.get(0, 2);
        double R21 = this.get(1, 0);
        double R22 = this.get(1, 1);
        double R23 = this.get(1, 2);
        double R31 = this.get(2, 0);
        double R32 = this.get(2, 1);
        double R33 = this.get(2, 2);
        double x = this.get(0, 3);
        double y = this.get(1, 3);
        double z = this.get(2, 3);
        double[][] inverse = new double[][] {{R11, R21, R31, -1 * (x * R11 + y * R21 + z * R31)}, {R12, R22, R32, -1 * (x * R12 + y * R22 + z * R32)}, {R13, R23, R33, -1 * (x * R13 + y * R23 + z * R33)}, {0, 0, 0, 1}};
        return new FourByFourMatrix(inverse);
    }
}

