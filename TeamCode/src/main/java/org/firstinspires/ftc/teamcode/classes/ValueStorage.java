package org.firstinspires.ftc.teamcode.classes;
import com.acmerobotics.roadrunner.geometry.Pose2d;
public class ValueStorage {
    public static double liftVm = 2500;
    public static double liftAm = 10000;
    public static double armVm = 2500;
    public static double armAm = 7500;
    public static double wristVm = 0.01;
    public static double wristAm = 0.01;
    public static double liftKp = 0.01;
    public static double liftKi = 0.02;
    public static double liftKd = 0.0002;
    public static double armKp = 0.02;
    public static double armKi = 0.02;
    public static double armKd = 0.0002;
    public static double liftMaxPower = 1;
    public static double armMaxPower = 0.75;
    public static double liftKf(double x, double v, double a) {
        return 0.1 + 0.00005 * x + 0.0002 * v + 0.00002 * a;
    }
    public static double armKf(double x, double v, double a) {
        return 0.0003 * v;
    }
    public static double clawClosed = 0.50;
    public static double clawOpen = 0.00;
    public static double liftGrab = 150;
    public static double liftLow = 650;
    public static double liftMid = 1150;
    public static double liftHigh = 1800;
    public static double liftGround = 70;
    public static double armDownFront = 0;
    public static double armDownBack = -650;
    public static double armDropFront = -80;
    public static double armDropBack = -570;
    public static double armGroundFront = -30;
    public static double armGroundBack = -620;
    public static double armWait = -325;
    public static double wristNeutral = 0;
    public static double wristDropFront = 0;
    public static double wristDropBack = 0;
    public static double grabAdjustIncrement = 80;
    public static double grabAdjustMax = 320;
    public static double grabHeight = 300;
    public static int signalMinCount = 10;
    public static Pose2d lastPose = new Pose2d(0, 0, 0);
}