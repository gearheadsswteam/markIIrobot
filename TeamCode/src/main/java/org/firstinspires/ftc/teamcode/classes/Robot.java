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
    public TrapezoidalProfile liftProfile;
    public MotionProfile armProfile;
    public MotionProfile wristProfile;
    public void init(HardwareMap hwMap) {
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
        PhotonCore.enable();
        IMU.Parameters parameters = new IMU.Parameters(new RevHubOrientationOnRobot(
                RevHubOrientationOnRobot.LogoFacingDirection.RIGHT,
                RevHubOrientationOnRobot.UsbFacingDirection.FORWARD));
        gyro.initialize(parameters);
    }
    public double heading() {
        return gyro.getRobotOrientation(AxesReference.INTRINSIC, AxesOrder.ZYX, AngleUnit.RADIANS).firstAngle;
    }
    public void update(double time) {
        liftPidf.set(liftProfile.getX(time));
        armPidf.set(armProfile.getX(time));
        liftPidf.update(time, liftL.getCurrentPosition() + liftR.getCurrentPosition(), liftProfile.getV(time), liftProfile.getA(time));
        armPidf.update(time,liftL.getCurrentPosition() - liftR.getCurrentPosition(), armProfile.getV(time), armProfile.getA(time));
        liftL.setPower(liftPidf.get() + armPidf.get());
        liftR.setPower(liftPidf.get() - armPidf.get());
    }
    public void setDrivePowers(double flPower, double frPower, double blPower, double brPower) {
        fl.setPower(flPower);
        fr.setPower(frPower);
        bl.setPower(blPower);
        br.setPower(brPower);
    }
}
