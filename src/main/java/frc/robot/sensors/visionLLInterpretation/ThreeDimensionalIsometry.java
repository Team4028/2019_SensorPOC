package frc.robot.sensors.visionLLInterpretation;

import frc.robot.auton.pathfollowing.motion.RigidTransform;
import frc.robot.auton.pathfollowing.motion.Rotation;
import frc.robot.auton.pathfollowing.motion.Translation;

public class ThreeDimensionalIsometry{
    private double x;
    private double y;
    private double z;
    private double alpha;
    private double beta;
    private double gamma;

    private FourByFourMatrix transformationMatrix;

    public ThreeDimensionalIsometry(double ex, double why, double zee, double a, double b, double c){
        x = ex;
        y = why;
        z = zee;
        alpha = a;
        beta = b;
        gamma = c;
        transformationMatrix = FourByFourMatrix.zAxisRotationMatrix(gamma).applyLinearTransformation(FourByFourMatrix.yAxisRotationMatrix(beta)).applyLinearTransformation(FourByFourMatrix.xAxisRotationMatrix(alpha)).applyLinearTransformation(FourByFourMatrix.translationTransformationMatrix(x, y, z));
    }

    public static ThreeDimensionalIsometry fromMatrix(FourByFourMatrix mat){
        double R11 = mat.get(0, 0);
        double R12 = mat.get(0, 1);
        double R13 = mat.get(0, 2);
        double R21 = mat.get(1, 0);
        double R22 = mat.get(1, 1);
        double R23 = mat.get(1, 2);
        double R31 = mat.get(2, 0);
        double R32 = mat.get(2, 1);
        double R33 = mat.get(2, 2);
        double a = 0;
        double b = 0;
        double c = 0;
        if (R31 != 1){
            a = -2 * Math.asin(R31);
            b = Math.atan2(R32/Math.cos(a), R33/Math.cos(a));
            c = Math.atan2(R21/Math.cos(a), R11/Math.cos(a));
        } else if (R31 == -1){
            a = Math.PI/2;
            b = Math.atan2(R12, R13);
        } else {
            a = -1 * Math.PI/2;
            b = Math.atan2(-1 * R12, -1 * R13);
        }
        double ex = mat.get(0, 3);
        double why = mat.get(1, 3);
        double zee = mat.get(2, 3);
        return new ThreeDimensionalIsometry(ex, why, zee, a, b, c);
    }

    public ThreeDimensionalIsometry applyIsometricTransformation(ThreeDimensionalIsometry other){
        return ThreeDimensionalIsometry.fromMatrix(this.transformationMatrix.applyLinearTransformation(other.transformationMatrix));
    }

    public ThreeDimensionalIsometry inverse(){
        return ThreeDimensionalIsometry.fromMatrix(this.getTransformationMatrix().invertAsIsometry());
    }

    public RigidTransform getXYIsometry(){
        return new RigidTransform(new Translation(x, y), Rotation.fromRadians(gamma));
    }

    public RigidTransform getXZIsometry(){
        return new RigidTransform(new Translation(x, z), Rotation.fromRadians(beta));
    }

    public RigidTransform getYZIsometry(){
        return new RigidTransform(new Translation(y, z), Rotation.fromRadians(alpha));
    }

    ///Getters

    public double getX(){
        return x;
    }

    public double getY(){
        return y;
    }

    public double getZ(){
        return z;
    }

    public double getAlpha(){
        return alpha;
    }

    public double getBeta(){
        return beta;
    }

    public double getGamme(){
        return gamma;
    }

    public FourByFourMatrix getTransformationMatrix(){
        return transformationMatrix;
    }
    
}