package org.firstinspires.ftc.teamcode.classes;
import com.acmerobotics.roadrunner.geometry.Pose2d;
import static com.qualcomm.robotcore.util.Range.*;
public class ValueStorage {
    public static double liftVm = 4000;
    public static double liftAm = 7500;
    public static double armVm = 2500;
    public static double armAm = 5000;
    public static double wristVm = 0;
    public static double wristAm = 0;
    public static double liftKp = 0.005;
    public static double liftKi = 0;
    public static double liftKd = 0;
    public static double armKp = 0.01;
    public static double armKi = 0;
    public static double armKd = 0;
    public static double liftKf(double x, double v, double a) {
        return 0.1 + 0.00005 * x + 0 * v + 0 * a;
    }
    public static double armKf(double x, double v, double a) {
        return 0;
    }
    public static int signalMinCount = 10;
    public static int side = sides.RED;
    public static class sides {
        public static final int RED = 1;
        public static final int BLUE = -1;
    }
    public static Pose2d lastPose = new Pose2d(0, 0, 0);
}