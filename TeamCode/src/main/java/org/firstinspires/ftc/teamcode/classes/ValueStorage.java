package org.firstinspires.ftc.teamcode.classes;
import static com.qualcomm.robotcore.util.Range.*;
import com.acmerobotics.roadrunner.geometry.Pose2d;
public class ValueStorage {
    public static double liftVm = 3000;
    public static double liftAm = 20000;
    public static double armVm = 1500;
    public static double armAm = 15000;
    public static double liftKp = 0.01;
    public static double liftKi = 0.01;
    public static double liftKd = 0.0002;
    public static double armKp = 0.02;
    public static double armKi = 0.02;
    public static double armKd = 0.0002;
    public static double liftMaxPower = 1;
    public static double armMaxPower = 0.5;
    public static double liftKf(double x, double v, double a) {
        return 0.06 + 0.00004 * x + 0.0002 * v + 0.00002 * a;
    }
    public static double armKf(double x, double v, double a) {
        return 0.0003 * v;
    }
    public static double clawClosed = 0.55;
    public static double clawOpen = 0.22;
    public static double liftGrab = 200;
    public static double liftLow = 700;
    public static double liftMid = 1425;
    public static double liftHigh = 2150;
    public static double liftGround = 80;
    public static double armDownFront = 0;
    public static double armDownBack = -760;
    public static double armDropFront = -90;
    public static double armDropBack = -670;
    public static double armGroundFront = -40;
    public static double armGroundBack = -720;
    public static double armWait = -380;
    public static double grabAdjustIncrement = 100;
    public static double grabAdjustMax = 400;
    public static double grabHeight = 400;
    public static double liftAdjustIncrement = 15;
    public static double liftAdjustMax = 2500;
    public static double armAdjustFront(double liftX) {
        if (liftX < (liftGround + liftLow) / 2) {
            return armGroundFront;
        } else if (liftX < liftLow) {
            return scale(liftX, (liftGround + liftLow) / 2, liftLow, armGroundFront, armDropFront);
        } else {
            return armDropFront;
        }
    }
    public static double armAdjustBack(double liftX) {
        if (liftX < (liftGround + liftLow) / 2) {
            return armGroundBack;
        } else if (liftX < liftLow) {
            return scale(liftX, (liftGround + liftLow) / 2, liftLow, armGroundBack, armDropBack);
        } else {
            return armDropBack;
        }
    }
    public static double currentThreshold = 1000;
    public static int signalMinCount = 10;
    public static Pose2d lastPose = new Pose2d(0, 0, 0);
}