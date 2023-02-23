package org.firstinspires.ftc.teamcode.autonomous;
import static org.firstinspires.ftc.teamcode.classes.ValueStorage.*;
import com.acmerobotics.dashboard.FtcDashboard;
import com.acmerobotics.dashboard.telemetry.MultipleTelemetry;
import com.acmerobotics.roadrunner.geometry.Pose2d;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.util.ElapsedTime;
import org.firstinspires.ftc.teamcode.classes.Robot;
import org.firstinspires.ftc.teamcode.classes.SignalDetector;
public abstract class AbstractAutonomous extends LinearOpMode {
    public Robot robot = new Robot();
    SignalDetector detector;
    int runCase = 1;
    int caseDetected = 1;
    int caseDetectionLength = 0;
    ElapsedTime clock = new ElapsedTime();
    double time = 0;
    @Override
    public void runOpMode() {
        telemetry = new MultipleTelemetry(telemetry, FtcDashboard.getInstance().getTelemetry());
        robot.init(hardwareMap, 0, armDownFront, wristNeutral);
        //detector = new SignalDetector(hardwareMap);
        //detector.init();
        initialize();
        robot.resetLift();
        robot.setLiftPos(clock.seconds() + 0.5, 0, armDropFront, wristDropFront);
        robot.claw.setPosition(clawClosed);
        while (!isStarted() && !isStopRequested()) {
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
            robot.update(clock.seconds());
            telemetry.addData("Case Detected", caseDetected);
            telemetry.addData("Case to Run", runCase);
            telemetry.update();
        }
        //detector.end();
        robot.drive.setPoseEstimate(initPose());
        run();
        lastPose = robot.drive.getPoseEstimate();
    }
    public abstract void initialize();
    public abstract void run();
    public abstract Pose2d initPose();
}
