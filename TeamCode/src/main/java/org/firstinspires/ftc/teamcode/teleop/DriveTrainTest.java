package org.firstinspires.ftc.teamcode.teleop;

import static org.firstinspires.ftc.teamcode.classes.ValueStorage.armDownBack;
import static org.firstinspires.ftc.teamcode.classes.ValueStorage.armDownFront;
import static org.firstinspires.ftc.teamcode.classes.ValueStorage.clawOpen;
import static org.firstinspires.ftc.teamcode.classes.ValueStorage.liftGrab;
import static org.firstinspires.ftc.teamcode.classes.ValueStorage.wristNeutral;

import com.acmerobotics.dashboard.FtcDashboard;
import com.acmerobotics.dashboard.telemetry.MultipleTelemetry;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.util.ElapsedTime;

import org.firstinspires.ftc.teamcode.classes.Robot;

@TeleOp(name = "DriveTrainTest")
public class DriveTrainTest extends LinearOpMode {

    Robot robot = new Robot();
    ElapsedTime clock = new ElapsedTime();

    public void runOpMode() {
        telemetry = new MultipleTelemetry(telemetry, FtcDashboard.getInstance().getTelemetry());

        robot.init(hardwareMap, 0, armDownFront, wristNeutral);

        while (!isStarted() && !isStopRequested()) {
            robot.update(clock.seconds());
        }
        while (opModeIsActive() && !isStopRequested()) {
            double powerFront = -gamepad1.left_stick_y;
           // robot.bl.setPower(power);
           // robot.br.setPower(power);
            robot.fl.setPower(powerFront);
            robot.fr.setPower(powerFront);

            double powerRear = -gamepad1.right_stick_y;
             robot.bl.setPower(powerRear);
             robot.br.setPower(powerRear);
           // robot.fl.setPower(powerFront);
            //robot.fr.setPower(powerFront);
        }
    }
}
