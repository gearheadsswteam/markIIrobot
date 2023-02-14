package teamcode.autonomous;

import static org.firstinspires.ftc.teamcode.classes.ValueStorage.armDownFront;
import static org.firstinspires.ftc.teamcode.classes.ValueStorage.armDropFront;
import static org.firstinspires.ftc.teamcode.classes.ValueStorage.clawClosed;
import static org.firstinspires.ftc.teamcode.classes.ValueStorage.lastPose;
import static org.firstinspires.ftc.teamcode.classes.ValueStorage.wristDropFront;
import static org.firstinspires.ftc.teamcode.classes.ValueStorage.wristNeutral;

import com.acmerobotics.roadrunner.geometry.Pose2d;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.util.ElapsedTime;

import org.firstinspires.ftc.teamcode.classes.Robot;
import org.firstinspires.ftc.teamcode.classes.SignalDetector;

public abstract class AbstractAutonomous extends LinearOpMode {
    public Robot robot = new Robot();
    SignalDetector detector;
    int runCase = 2;
    int caseDetected = 2;
    int caseDetectionLength = 0;
    ElapsedTime clock = new ElapsedTime();
    double time;
    @Override
    public void runOpMode() {
        robot.init(hardwareMap, 0, armDownFront, wristNeutral);
        //detector = new SignalDetector(hardwareMap);
        //detector.init();
        robot.drive.setPoseEstimate(initPose());
        initialize();
        robot.liftL.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        robot.liftR.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        robot.liftL.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        robot.liftR.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        robot.setLiftPos(clock.seconds() + 0.5, 0, armDropFront, wristDropFront);
        robot.claw.setPosition(clawClosed);
        while (!isStarted() && !isStopRequested()) {
            time = clock.seconds();
            /*
            if (detector.getCaseDetected() == caseDetected) {
                caseDetectionLength++;
            } else if (detector.getCaseDetected() > 0) {
                caseDetected = detector.getCaseDetected();
                caseDetectionLength = 1;
            }
            if (caseDetectionLength >= signalMinCount) {
                runCase = caseDetected;
            }
            */
            robot.update(time);
            telemetry.addData("Case Detected", caseDetected);
            telemetry.addData("Case to Run", runCase);
            telemetry.update();
        }
        //detector.end();
        run();
        lastPose = robot.drive.getPoseEstimate();
    }
    public abstract void initialize();
    public abstract void run();
    public abstract Pose2d initPose();
}
