package org.firstinspires.ftc.teamcode.classes;
import static java.lang.Math.*;
import static org.firstinspires.ftc.teamcode.classes.ValueStorage.*;
import androidx.annotation.GuardedBy;
import com.outoftheboxrobotics.photoncore.PhotonCore;
import com.qualcomm.hardware.lynx.LynxModule;
import com.qualcomm.hardware.lynx.LynxNackException;
import com.qualcomm.hardware.lynx.commands.core.LynxGetADCCommand;
import com.qualcomm.hardware.lynx.commands.core.LynxGetADCResponse;
import com.qualcomm.hardware.rev.RevHubOrientationOnRobot;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
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
    public LynxModule chub;
    private double heading = 0;
    private final Object gyroLock = new Object();
    @GuardedBy("gyroLock")
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
    public void init(HardwareMap hwMap, double liftX, double armX, boolean usingGyro) {
        drive = new SampleMecanumDrive(hwMap);
        fl = hwMap.get(DcMotorEx.class, "fl");
        fr = hwMap.get(DcMotorEx.class, "fr");
        bl = hwMap.get(DcMotorEx.class, "bl");
        br = hwMap.get(DcMotorEx.class, "br");
        liftL = hwMap.get(DcMotorEx.class, "liftL");
        liftR = hwMap.get(DcMotorEx.class, "liftR");
        claw = hwMap.get(Servo.class, "claw");
        gyro = hwMap.get(IMU.class, "gyro");
        chub = PhotonCore.CONTROL_HUB;
        fl.setDirection(DcMotorSimple.Direction.REVERSE);
        bl.setDirection(DcMotorSimple.Direction.REVERSE);
        liftL.setDirection(DcMotorSimple.Direction.REVERSE);
        fl.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        fr.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        bl.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        br.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        liftProfile = new DelayProfile(0, liftX, 0,0);
        armProfile = new DelayProfile(0, armX, 0, 0);
        PhotonCore.experimental.setMaximumParallelCommands(8);
        PhotonCore.enable();
        if (usingGyro) {
            synchronized (gyroLock) {
                IMU.Parameters parameters = new IMU.Parameters(new RevHubOrientationOnRobot(
                        RevHubOrientationOnRobot.LogoFacingDirection.RIGHT,
                        RevHubOrientationOnRobot.UsbFacingDirection.UP));
                gyro.initialize(parameters);
                gyro.resetYaw();
            }
        }
        claw.setPosition(clawOpen);
    }
    public double restTime() {
        return max(liftProfile.getTf(), armProfile.getTf());
    }
    public double servoCurrent() {
        LynxGetADCCommand command = new LynxGetADCCommand(chub, LynxGetADCCommand.Channel.SERVO_CURRENT, LynxGetADCCommand.Mode.ENGINEERING);
        try {
            LynxGetADCResponse response = command.sendReceive();
            return response.getValue();
        } catch (InterruptedException|RuntimeException|LynxNackException e) {
            return 0;
        }
    }
    public double getHeading() {
        return heading;
    }
    public void setLiftPos(double time, double liftX, double armX) {
        double t1 = liftProfile.extendTrapezoidal(liftVm, liftAm, time, liftX, 0).getTf() - time;
        double t2 = armProfile.extendTrapezoidal(armVm, armAm, time, armX, 0).getTf() - time;
        double f1 = 0;
        double f2 = 0;
        if (t1 != 0) {
            f1 = min(1, 1 / (t2 * armMaxPower / t1 + liftMaxPower));
        }
        if (t2 != 0) {
            f2 = min(1, 1 / (t1 * liftMaxPower / t2 + armMaxPower));
        }
        liftProfile = liftProfile.extendTrapezoidal(liftVm * f1, liftAm * f1 * f1, time, liftX, 0);
        armProfile = armProfile.extendTrapezoidal(armVm * f2, armAm * f2 * f2, time, armX, 0);
    }
    public void update(double time) {
        liftPidf.set(liftProfile.getX(time));
        armPidf.set(armProfile.getX(time));
        double leftX = liftL.getCurrentPosition();
        double rightX = liftR.getCurrentPosition();
        liftPidf.update(time, leftX + rightX, liftProfile.getV(time), liftProfile.getA(time));
        armPidf.update(time,leftX - rightX, armProfile.getV(time), armProfile.getA(time));
        liftL.setPower(liftPidf.get() + armPidf.get());
        liftR.setPower(liftPidf.get() - armPidf.get());
    }
    public void setDrivePowers(double flPower, double frPower, double blPower, double brPower) {
        fl.setPower(flPower);
        fr.setPower(frPower);
        bl.setPower(blPower);
        br.setPower(brPower);
    }
    public void resetLift() {
        liftL.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        liftR.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        liftL.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        liftR.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
    }
    public void startGyro(LinearOpMode opMode) {
        Thread gyroThread = new Thread(() -> {
            opMode.waitForStart();
            while (opMode.opModeIsActive() && !opMode.isStopRequested()) {
                synchronized (gyroLock) {
                    heading = gyro.getRobotOrientation(AxesReference.INTRINSIC, AxesOrder.ZYX, AngleUnit.RADIANS).firstAngle;
                }
            }
        });
        gyroThread.start();
    }
}
