package org.firstinspires.ftc.teamcode.teleop;
import static org.firstinspires.ftc.teamcode.classes.ValueStorage.*;
import com.acmerobotics.dashboard.FtcDashboard;
import com.acmerobotics.dashboard.config.Config;
import com.acmerobotics.dashboard.telemetry.MultipleTelemetry;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotor;
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
    double armSetPoint = 0;
    public static double LIFT_KV = 0;
    public static double LIFT_KA = 0;
    public static double ARM_KV = 0.0003;
    public static double ARM_KA = 0;
    public static double LIFT_KP = 0.005;
    public static double LIFT_KI = 0;
    public static double LIFT_KD = 0;
    public static double ARM_KP = 0.01;
    public static double ARM_KI = 0;
    public static double ARM_KD = 0;
    public static double LIFT_VM = 4000;
    public static double LIFT_AM = 7500;
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
    boolean lbPressed = false;
    boolean lbReleased = false;
    boolean rbPressed = false;
    boolean rbReleased = false;
    ElapsedTime clock = new ElapsedTime();
    PidfController liftPid = new PidfController(LIFT_KP, LIFT_KI, LIFT_KD) {
        @Override
        public double kf(double x, double v, double a) {
            return 0.1 + 0.00005 * x;
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
        robot.init(hardwareMap, 0, armDownBack, wristNeutral);
        robot.liftL.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        robot.liftR.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        robot.liftL.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        robot.liftR.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
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
            if (gamepad1.left_bumper) {
                lbPressed = lbReleased;
                lbReleased = false;
            } else {
                lbPressed = false;
                lbReleased = true;
            }
            if (gamepad1.right_bumper) {
                rbPressed = rbReleased;
                rbReleased = false;
            } else {
                rbPressed = false;
                rbReleased = true;
            }
            time = clock.seconds();
            if (aPressed && time > robot.restTime()) {
                robot.setLiftPos(time, 2000, 0, wristNeutral);
            } else if (bPressed && time > robot.restTime()) {
                robot.setLiftPos(time, 2000, -650, wristNeutral);
            } else if (yPressed && time > robot.restTime()) {
                robot.setLiftPos(time, 0, -650, wristNeutral);
            } else if (xPressed && time > robot.restTime()) {
                robot.setLiftPos(time, 0, 0, wristNeutral);
            }
            /*
            liftPid.setConstants(LIFT_KP, LIFT_KI, LIFT_KD);
            armPid.setConstants(ARM_KP, ARM_KI, ARM_KD);
            liftPid.set(liftProfile.getX(time));
            liftPid.update(time, robot.liftL.getCurrentPosition() + robot.liftR.getCurrentPosition(), liftProfile.getV(time), liftProfile.getA(time));
            armPid.set(armProfile.getX(time));
            armPid.update(time, robot.liftL.getCurrentPosition() - robot.liftR.getCurrentPosition(), armProfile.getV(time), armProfile.getA(time));
            double liftF = liftPid.get() + LIFT_KV * liftProfile.getV(time) + LIFT_KA * liftProfile.getA(time);
            double armF = armPid.get() + ARM_KV * armProfile.getV(time) + LIFT_KA * armProfile.getA(time);
            robot.liftL.setPower(liftF + armF);
            robot.liftR.setPower(liftF - armF);
            */
            //telemetry.addData("Left Power", liftF + armF);
            //telemetry.addData("Right Power", liftF - armF);
            robot.update(time);
            telemetry.addData("Lift Set Point", robot.liftProfile.getX(time));
            telemetry.addData("Lift Position", robot.liftL.getCurrentPosition() + robot.liftR.getCurrentPosition());
            telemetry.addData("Arm Set Point", robot.armProfile.getX(time));
            telemetry.addData("Arm Position", robot.liftL.getCurrentPosition() - robot.liftR.getCurrentPosition());
            telemetry.update();
        }
    }
}