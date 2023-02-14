package org.firstinspires.ftc.teamcode.classes;
import static java.lang.Math.*;
import static org.firstinspires.ftc.teamcode.classes.ValueStorage.*;
import com.outoftheboxrobotics.photoncore.PhotonCore;
import com.qualcomm.hardware.rev.RevColorSensorV3;
import com.qualcomm.hardware.rev.RevHubOrientationOnRobot;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.hardware.IMU;
import com.qualcomm.robotcore.hardware.Servo;
import org.firstinspires.ftc.robotcore.external.navigation.AngleUnit;
import org.firstinspires.ftc.robotcore.external.navigation.AxesOrder;
import org.firstinspires.ftc.robotcore.external.navigation.AxesReference;
import org.firstinspires.ftc.teamcode.drive.SampleMecanumDrive;
public class Robot {
    public SampleMecanumDrive drive;
    public DcMotorEx fl;
    public DcMotorEx fr;
    public DcMotorEx bl;
    public DcMotorEx br;
    public DcMotorEx liftL;
    public DcMotorEx liftR;
    public Servo claw;
    public Servo wrist;
    public IMU gyro;
    PidfController liftPidf = new PidfController(liftKp, liftKi, liftKd) {
        @Override
        public double kf(double x, double v, double a) {
            return liftKf(x, v, a);
        }
    };
    PidfController armPidf = new PidfController(armKp, armKi, armKd) {
        @Override
        public double kf(double x, double v, double a) {
            return armKf(x, v, a);
        }
    };
    public MotionProfile liftProfile;
    public MotionProfile armProfile;
    public MotionProfile wristProfile;
    public void init(HardwareMap hwMap, double liftX, double armX, double wristX) {
        drive = new SampleMecanumDrive(hwMap);
        fl = hwMap.get(DcMotorEx.class, "fl");
        fr = hwMap.get(DcMotorEx.class, "fr");
        bl = hwMap.get(DcMotorEx.class, "bl");
        br = hwMap.get(DcMotorEx.class, "br");
        liftL = hwMap.get(DcMotorEx.class, "liftL");
        liftR = hwMap.get(DcMotorEx.class, "liftR");
        wrist = hwMap.get(Servo.class, "wrist");
        claw = hwMap.get(Servo.class, "claw");
        gyro = hwMap.get(IMU.class, "gyro");
        fl.setDirection(DcMotorSimple.Direction.REVERSE);
        bl.setDirection(DcMotorSimple.Direction.REVERSE);
        liftL.setDirection(DcMotorSimple.Direction.REVERSE);
        fl.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        fr.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        bl.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        br.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        liftProfile = new DelayProfile(0, liftX, 0,0);
        armProfile = new DelayProfile(0, armX, 0, 0);
        wristProfile = new DelayProfile(0, wristX, 0, 0);
        PhotonCore.enable();
        IMU.Parameters parameters = new IMU.Parameters(new RevHubOrientationOnRobot(
                RevHubOrientationOnRobot.LogoFacingDirection.RIGHT,
                RevHubOrientationOnRobot.UsbFacingDirection.UP));
        gyro.initialize(parameters);
        claw.setPosition(clawOpen);
    }
    public double heading() {
        return gyro.getRobotOrientation(AxesReference.INTRINSIC, AxesOrder.ZYX, AngleUnit.RADIANS).firstAngle;
    }
    public double restTime() {
        return max(max(liftProfile.getTf(), armProfile.getTf()), wristProfile.getTf());
    }
    public void setLiftPos(double time, double liftX, double armX, double wristPos) {
        double t1 = liftProfile.extendTrapezoidal(liftAm, liftVm, time, liftX, 0).getTf() - time;
        double t2 = armProfile.extendTrapezoidal(armAm, armVm, time, armX, 0).getTf() - time;
        double f1;
        double f2;
        if (t1 == 0) {
            f1 = 0;
            f2 = 1;
        } else if (t2 == 0) {
            f1 = 1;
            f2 = 0;
        } else {
            f1 = 1 / (t1 * armMaxPower / t2 + liftMaxPower);
            f2 = 1 / (t2 * liftMaxPower / t1 + armMaxPower);
        }
        liftProfile = liftProfile.extendTrapezoidal(liftAm * f1, liftVm * f1, time, liftX, 0);
        armProfile = armProfile.extendTrapezoidal(armAm * f2, armVm * f2, time, armX, 0);
        wristProfile = wristProfile.extendTrapezoidal(wristAm, wristVm, time, wristPos, 0);
    }
    public void update(double time) {
        liftPidf.set(liftProfile.getX(time));
        armPidf.set(armProfile.getX(time));
        liftPidf.update(time, liftL.getCurrentPosition() + liftR.getCurrentPosition(), liftProfile.getV(time), liftProfile.getA(time));
        armPidf.update(time,liftL.getCurrentPosition() - liftR.getCurrentPosition(), armProfile.getV(time), armProfile.getA(time));
        liftL.setPower(liftPidf.get() + armPidf.get());
        liftR.setPower(liftPidf.get() - armPidf.get());
        wrist.setPosition(wristProfile.getX(time));
    }
    public void setDrivePowers(double flPower, double frPower, double blPower, double brPower) {
        fl.setPower(flPower);
        fr.setPower(frPower);
        bl.setPower(blPower);
        br.setPower(brPower);
    }
}
