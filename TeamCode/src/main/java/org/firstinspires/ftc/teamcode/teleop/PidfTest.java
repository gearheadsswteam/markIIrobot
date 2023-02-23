package org.firstinspires.ftc.teamcode.teleop;
import static org.firstinspires.ftc.teamcode.classes.ValueStorage.*;
import static java.lang.Math.*;
import com.acmerobotics.dashboard.FtcDashboard;
import com.acmerobotics.dashboard.config.Config;
import com.acmerobotics.dashboard.telemetry.MultipleTelemetry;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.util.ElapsedTime;
import org.firstinspires.ftc.teamcode.classes.PidfController;
import org.firstinspires.ftc.teamcode.classes.Robot;
import org.firstinspires.ftc.teamcode.classes.TrapezoidalProfile;
@Config
@TeleOp(name = "PidfTest")
public class PidfTest extends LinearOpMode {
    Robot robot = new Robot();
    double time = 0;
    double liftSetPoint = 0;
    double armSetPoint = armDownFront;
    double t1;
    double t2;
    double f1;
    double f2;
    public static double LIFT_MP = 1;
    public static double ARM_MP = 0.5;
    public static double LIFT_KGS = 0.1;
    public static double LIFT_KGD = 0.00005;
    public static double LIFT_KV = 0.0002;
    public static double LIFT_KA = 0.00001;
    public static double ARM_KV = 0.0003;
    public static double ARM_KA = 0;
    public static double LIFT_KP = 0.01;
    public static double LIFT_KI = 0;
    public static double LIFT_KD = 0.0002;
    public static double ARM_KP = 0.02;
    public static double ARM_KI = 0.01;
    public static double ARM_KD = 0.0003;
    public static double LIFT_VM = 3000;
    public static double LIFT_AM = 10000;
    public static double ARM_VM = 2500;
    public static double ARM_AM = 5000;
    boolean aPressed = false;
    boolean aReleased = true;
    boolean bPressed = false;
    boolean bReleased = true;
    boolean xPressed = false;
    boolean xReleased = false;
    boolean yPressed = false;
    boolean yReleased = false;
    ElapsedTime clock = new ElapsedTime();
    PidfController liftPid = new PidfController(LIFT_KP, LIFT_KI, LIFT_KD) {
        @Override
        public double kf(double x, double v, double a) {
            return 0;
        }
    };
    PidfController armPid = new PidfController(ARM_KP, ARM_KI, ARM_KD) {
        @Override
        public double kf(double x, double v, double a) {
            return 0;
        }
    };
    TrapezoidalProfile liftProfile = new TrapezoidalProfile(LIFT_VM, LIFT_AM, 0, liftSetPoint, 0, liftSetPoint, 0);
    TrapezoidalProfile armProfile = new TrapezoidalProfile(ARM_VM, ARM_AM, 0, armSetPoint, 0, armSetPoint, 0);
    @Override
    public void runOpMode() {
        telemetry = new MultipleTelemetry(telemetry, FtcDashboard.getInstance().getTelemetry());
        robot.init(hardwareMap, 0, armDownFront, wristNeutral);
        robot.resetLift();
        waitForStart();
        while (opModeIsActive() && !isStopRequested()) {
            if (gamepad1.a) {
                aPressed = aReleased;
                aReleased = false;
            } else {
                aPressed = false;
                aReleased = true;
            }
            if (gamepad1.b) {
                bPressed = bReleased;
                bReleased = false;
            } else {
                bPressed = false;
                bReleased = true;
            }
            if (gamepad1.x) {
                xPressed = xReleased;
                xReleased = false;
            } else {
                xPressed = false;
                xReleased = true;
            }
            if (gamepad1.y) {
                yPressed = yReleased;
                yReleased = false;
            } else {
                yPressed = false;
                yReleased = true;
            }
            time = clock.seconds();
            if (aPressed) {
                liftSetPoint = 0;
                armSetPoint = armDownFront;
            } else if (bPressed) {
                liftSetPoint = liftHigh;
                armSetPoint = armDownFront;
            } else if (yPressed) {
                liftSetPoint = liftHigh;
                armSetPoint = armDownBack;
            } else if (xPressed) {
                liftSetPoint = 0;
                armSetPoint = armDownBack;
            }
            if ((aPressed || bPressed || xPressed || yPressed) && time > liftProfile.getTf() && time > armProfile.getTf()) {
                t1 = liftProfile.extendTrapezoidal(LIFT_VM, LIFT_AM, time, liftSetPoint, 0).getTf() - time;
                t2 = armProfile.extendTrapezoidal(ARM_VM, ARM_AM, time, armSetPoint, 0).getTf() - time;
                f1 = 0;
                f2 = 0;
                if (t1 != 0) {
                    f1 = min(1, 1 / (t2 * ARM_MP / t1 + LIFT_MP));
                }
                if (t2 != 0) {
                    f2 = min(1, 1 / (t1 * ARM_MP / t2 + LIFT_MP));
                }
                liftProfile = liftProfile.extendTrapezoidal(LIFT_VM * f1, LIFT_AM * f1 * f1, time, liftSetPoint, 0);
                armProfile = armProfile.extendTrapezoidal(ARM_VM * f2, ARM_AM * f2 * f2, time, armSetPoint, 0);
                telemetry.addData("Lift Time:", liftProfile.getTf() - time);
                telemetry.addData("Arm Time:", armProfile.getTf() - time);
            }
            liftPid.setConstants(LIFT_KP, LIFT_KI, LIFT_KD);
            armPid.setConstants(ARM_KP, ARM_KI, ARM_KD);
            double leftX = robot.liftL.getCurrentPosition();
            double rightX = robot.liftR.getCurrentPosition();
            liftPid.set(liftProfile.getX(time));
            liftPid.update(time, leftX + rightX, 0, 0);
            armPid.set(armProfile.getX(time));
            armPid.update(time, leftX - rightX, 0, 0);
            double liftF = liftPid.get() + LIFT_KGS + LIFT_KGD * (leftX + rightX) + LIFT_KV * liftProfile.getV(time) + LIFT_KA * liftProfile.getA(time);
            double armF = armPid.get() + ARM_KV * armProfile.getV(time) + ARM_KA * armProfile.getA(time);
            robot.liftL.setPower(liftF + armF);
            robot.liftR.setPower(liftF - armF);
            telemetry.addData("Left Power", liftF + armF);
            telemetry.addData("Right Power", liftF - armF);
            telemetry.addData("Lift Set Point", liftProfile.getX(time));
            telemetry.addData("Lift Position", leftX + rightX);
            telemetry.addData("Arm Set Point", armProfile.getX(time));
            telemetry.addData("Arm Position", leftX - rightX);
            telemetry.update();
        }
    }
}