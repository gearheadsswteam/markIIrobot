package teamcode.classes;

import static java.lang.Math.abs;
import static java.lang.Math.pow;
import static java.lang.Math.sqrt;

import org.firstinspires.ftc.teamcode.classes.MotionProfile;

public class TrapezoidalProfile extends MotionProfile {
    double am;
    double vm;
    boolean flat;
    public TrapezoidalProfile(double vm, double am, double ti, double xi, double vi, double xf, double vf) {
        this.vm = vm;
        this.xi = xi;       
        this.vi = vi;
        this.ti = ti;
        this.xf = xf;
        this.vf = vf;
        this.flat = 2 * pow(vm, 2) - pow(vi, 2) - pow(vf, 2) < 2 * am * abs(xf - xi);
        if (abs(pow(vf, 2) - pow(vi, 2)) > 2 * am * abs(xf - xi)) {
            this.am = abs((pow(vf, 2) - pow(vi, 2)) / (2 * (xf - xi)));
        } else {
            this.am = am;
        }
        if (am != 0 && vm != 0) {
            if (xf > xi && flat) {
                this.tf = ti + (xf - xi) / vm + (pow(vm - vi, 2) + pow(vm - vf, 2)) / (2 * this.am * vm);
            } else if (xf > xi) {
                this.tf = ti + (2 * sqrt(am * (xf - xi) + (pow(vi, 2) + pow(vf, 2)) / 2) - vi - vf) / this.am;
            } else if (flat) {
                this.tf = ti + (xi - xf) / vm + (pow(vm + vi, 2) + pow(vm + vf, 2)) / (2 * this.am * vm);
            } else {
                this.tf = ti + (2 * sqrt(am * (xi - xf) + (pow(vi, 2) + pow(vf, 2)) / 2) + vi + vf) / this.am;
            }
        } else {
            this.tf = 0;
        }
    }
    @Override
    public double getX(double t) {
        if (t < ti) {
            return xi + vi * (t - ti);
        } if (t < tf) {
            if (xf > xi && flat) {
                if (t < ti + (vm - vi) / am) {
                    return xi + vi * (t - ti) + am * pow(t - ti, 2) / 2;
                } else if (t < tf - vm / am) {
                    return xi + vm * (t - ti) - pow(vm - vi, 2) / (2 * am);
                } else {
                    return xf - vf * (tf - t) - am * pow(tf - t, 2) / 2;
                }
            } else if (xf > xi) {
                if (t < (tf + ti) / 2 + (vf - vi) / (2 * am)) {
                    return xi + vi * (t - ti) + am * pow(t - ti, 2) / 2;
                } else {
                    return xf - vf * (tf - t) - am * pow(tf - t, 2) / 2;
                }
            } else if (flat) {
                if (t < ti + (vm + vi) / am) {
                    return xi + vi * (t - ti) - am * pow(t - ti, 2) / 2;
                } else if (t < tf - vm / am) {
                    return xi - vm * (t - ti) + pow(vm + vi, 2) / (2 * am);
                } else {
                    return xf - vf * (tf - t) + am * pow(tf - t, 2) / 2;
                }
            } else {
                if (t < (tf + ti) / 2 + (vi - vf)/ (2 * am)) {
                    return xi + vi * (t - ti) - am * pow(t - ti, 2) / 2;
                } else {
                    return xf - vf * (tf - t) + am * pow(tf - t, 2) / 2;
                }
            }
        } else {
            return xf + vf * (t - tf);
        }
    }
    @Override
    public double getV(double t) {
        if (t < ti) {
            return vi;
        } else if (t < tf) {
            if (xf > xi && flat) {
                if (t < ti + (vm - vi) / am) {
                    return vi + am * (t - ti);
                } else if (t < tf - vm / am) {
                    return vm;
                } else {
                    return vf + am * (tf - t);
                }
            } else if (xf > xi) {
                if (t < (tf + ti) / 2 + (vf - vi) / (2 * am)) {
                    return vi + am * (t - ti);
                } else {
                    return vf + am * (tf - t);
                }
            } else if (flat) {
                if (t < ti + (vm + vi) / am) {
                    return vi - am * (t - ti);
                } else if (t < tf - vm / am) {
                    return -vm;
                } else {
                    return vf - am * (tf - t);
                }
            } else {
                if (t < (tf + ti) / 2 + (vi - vf) / (2 * am)) {
                    return vi - am * (t - ti);
                } else {
                    return vf - am * (tf - t);
                }
            }
        } else {
            return vf;
        }
    }
    @Override
    public double getA(double t) {
        if (t < 0) {
            return vi;
        } else if (t < tf) {
            if (xf > xi && flat) {
                if (t < ti + (vm - vi) / am) {
                    return am;
                } else if (t < tf - vm / am) {
                    return 0;
                } else {
                    return -am;
                }
            } else if (xf > xi) {
                if (t < (tf + ti) / 2 + (vf - vi) / (2 * am)) {
                    return am;
                } else {
                    return -am;
                }
            } else if (flat) {
                if (t < ti + (vm + vi) / am) {
                    return -am;
                } else if (t < tf - vm / am) {
                    return 0;
                } else {
                    return am;
                }
            } else {
                if (t < (tf + ti) / 2 + (vi - vf) / (2 * am)) {
                    return -am;
                } else {
                    return am;
                }
            }
        } else {
            return 0;
        }
    }
    public org.firstinspires.ftc.teamcode.classes.TrapezoidalProfile extendTrapezoidal(double t, double xFinal, double vFinal) {
        return new org.firstinspires.ftc.teamcode.classes.TrapezoidalProfile(vm, am, t, getX(t), getV(t), xFinal, vFinal);
    }
    public org.firstinspires.ftc.teamcode.classes.TrapezoidalProfile extendTrapezoidal(double xFinal, double vFinal) {
        return new org.firstinspires.ftc.teamcode.classes.TrapezoidalProfile(vm, am, tf, xf, vf, xFinal, vFinal);
    }
}