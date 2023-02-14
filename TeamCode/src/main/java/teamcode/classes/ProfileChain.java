package teamcode.classes;
import org.firstinspires.ftc.teamcode.classes.MotionProfile;

import java.util.ArrayList;

public class ProfileChain extends MotionProfile {
    ArrayList<MotionProfile> profiles = new ArrayList<>();
    public ProfileChain(ArrayList<MotionProfile> profiles) {
        for (int i = 0; i < profiles.size(); i++) {
            if (i < profiles.size() - 1 && profiles.get(i).getX(profiles.get(i + 1).getTi()) != profiles.get(i + 1).xi) {
                throw new RuntimeException("PositionContinuityError");
            } else if (i < profiles.size() - 1 && profiles.get(i).getV(profiles.get(i + 1).getTi()) != profiles.get(i + 1).vi) {
                throw new RuntimeException("VelocityContinuityError");
            }  else if (i < profiles.size() - 1 && profiles.get(i).getTf() > profiles.get(i + 1).getTi()) {
                throw new RuntimeException("ProfileInterferenceError");
            }
            if (profiles.get(i) instanceof org.firstinspires.ftc.teamcode.classes.ProfileChain) {
                this.profiles.addAll(((org.firstinspires.ftc.teamcode.classes.ProfileChain) profiles.get(i)).getProfiles());
            } else {
                this.profiles.add(profiles.get(i));
            }
        }
        this.ti = this.profiles.get(0).getTi();
        this.xi = this.profiles.get(0).xi;
        this.vi = this.profiles.get(0).vi;
        this.tf = this.profiles.get(this.profiles.size() - 1).getTf();
        this.xf = this.profiles.get(this.profiles.size() - 1).xf;
        this.vf = this.profiles.get(this.profiles.size() - 1).vf;
    }
    public ProfileChain(MotionProfile profile) {
        this.profiles.add(profile);
        this.ti = profile.getTi();
        this.xi = profile.xi;
        this.vi = profile.vi;
        this.tf = profile.getTf();
        this.xf = profile.xf;
        this.vf = profile.vf;
    }
    @Override
    public double getX(double t) {
        for (int i = 0; i < profiles.size() - 1; i++) {
            if (t < profiles.get(i + 1).getTi()) {
                return profiles.get(i).getX(t);
            }
        }
        return profiles.get(profiles.size() - 1).getX(t);
    }
    @Override
    public double getV(double t) {
        for (int i = 0; i < profiles.size() - 1; i++) {
            if (t < profiles.get(i + 1).getTi()) {
                return profiles.get(i).getV(t);
            }
        }
        return profiles.get(profiles.size() - 1).getV(t);
    }
    @Override
    public double getA(double t) {
        for (int i = 0; i < profiles.size() - 1; i++) {
            if (t < profiles.get(i + 1).getTi()) {
                return profiles.get(i).getA(t);
            }
        }
        return profiles.get(profiles.size() - 1).getA(t);
    }
    public ArrayList<MotionProfile> getProfiles() {
        return profiles;
    }
    public org.firstinspires.ftc.teamcode.classes.ProfileChain add(MotionProfile newProfile) {
        profiles.add(newProfile);
        return new org.firstinspires.ftc.teamcode.classes.ProfileChain(profiles);
    }
    public org.firstinspires.ftc.teamcode.classes.ProfileChain addExtendTrapezoidal(double vm, double am, double t, double xf, double vf) {
        if (t < tf) {
            throw new RuntimeException("ProfileInterferenceError");
        }
        return add(extendTrapezoidal(vm, am, t, xf, vf));
    }
    public org.firstinspires.ftc.teamcode.classes.ProfileChain addExtendTrapezoidal(double vm, double am, double xf, double vf) {
        return add(extendTrapezoidal(vm, am, xf, vf));
    }
    public org.firstinspires.ftc.teamcode.classes.ProfileChain addExtendDelay(double t, double deltaT) {
        if (t < tf) {
            throw new RuntimeException("ProfileInterferenceError");
        }
        return add(extendDelay(t, deltaT));
    }
    public org.firstinspires.ftc.teamcode.classes.ProfileChain addExtendDelay(double deltaT) {
        return add(extendDelay(deltaT));
    }
}
