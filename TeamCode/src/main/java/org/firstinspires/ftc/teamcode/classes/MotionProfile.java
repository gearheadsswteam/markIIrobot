package org.firstinspires.ftc.teamcode.classes;

public abstract class MotionProfile {
    double xi;
    double vi;
    double ti;
    double xf;
    double tf;
    double vf;
    public abstract double getX(double t);
    public abstract double getV(double t);
    public abstract double getA(double t);
    public double getTf() {
        return tf;
    }
    public double getTi() {
        return ti;
    }
    public TrapezoidalProfile extendTrapezoidal(double vMax, double aMax, double t, double xFinal, double vFinal) {
        return new TrapezoidalProfile(vMax, aMax, t, getX(t), getV(t), xFinal, vFinal);
    }
    public TrapezoidalProfile extendTrapezoidal(double vMax, double aMax, double xFinal, double vFinal) {
        return extendTrapezoidal(vMax, aMax, tf, xFinal, vFinal);
    }
    public DelayProfile extendDelay(double t, double dt) {
        return new DelayProfile(t, getX(t), getV(t), dt);
    }
    public DelayProfile extendDelay(double dt) {
        return extendDelay(tf, dt);
    }
}
