package org.firstinspires.ftc.teamcode.teleop;
import static java.lang.Math.*;
import static com.qualcomm.robotcore.util.Range.*;
import static org.firstinspires.ftc.teamcode.classes.ValueStorage.*;
import com.acmerobotics.dashboard.FtcDashboard;
import com.acmerobotics.dashboard.telemetry.MultipleTelemetry;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.util.ElapsedTime;
import org.firstinspires.ftc.teamcode.classes.Robot;
import org.firstinspires.ftc.teamcode.classes.ValueStorage;
@TeleOp(name = "TwoDriver")
public class TeleopTwoDriver extends LinearOpMode {
    Robot robot = new Robot();
    int state = 0;
    double initialHeading = ValueStorage.lastPose.getHeading() + PI / 2;
    double moveAngle;
    double moveMagnitude;
    double turn;
    double time;
    double lastLiftX = liftHigh;
    double lastArmX = armDropFront;
    double lastWristX = wristNeutral;
    double grabAdjust = 0;
    boolean closeClaw = false;
    boolean grabbingBack = true;
    boolean waiting = false;
    boolean aPressed = false;
    boolean bPressed = false;
    boolean aReleased = true;
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
    @Override
    public void runOpMode() {
        telemetry = new MultipleTelemetry(telemetry, FtcDashboard.getInstance().getTelemetry());
        robot.init(hardwareMap, 0, armWait, wristNeutral);
        robot.startGyro(this);
        robot.setLiftPos(clock.seconds(), liftGrab, armDownBack, wristNeutral);
        while (!isStarted() && !isStopRequested()) {
            robot.update(clock.seconds());
        }
        while (opModeIsActive() && !isStopRequested()) {
            if (gamepad2.a) {
                aPressed = aReleased;
                aReleased = false;
            } else {
                aPressed = false;
                aReleased = true;
            }
            if (gamepad2.b) {
                bPressed = bReleased;
                bReleased = false;
            } else {
                bPressed = false;
                bReleased = true;
            }
            if (gamepad2.x) {
                xPressed = xReleased;
                xReleased = false;
            } else {
                xPressed = false;
                xReleased = true;
            }
            if (gamepad2.y) {
                yPressed = yReleased;
                yReleased = false;
            } else {
                yPressed = false;
                yReleased = true;
            }
            if (gamepad2.left_bumper) {
                lbPressed = lbReleased;
                lbReleased = false;
            } else {
                lbPressed = false;
                lbReleased = true;
            }
            if (gamepad2.right_bumper) {
                rbPressed = rbReleased;
                rbReleased = false;
            } else {
                rbPressed = false;
                rbReleased = true;
            }
            if (gamepad1.ps) {
                initialHeading = -robot.getHeading();
            }
            time = clock.seconds();
            if (state == 0 && time > robot.restTime()) {
                if (grabbingBack) {
                    if (rbPressed) {
                        closeClaw = true;
                        robot.setLiftPos(time, grabAdjust, armDownBack, wristNeutral);
                        state = 1;
                    } else if (gamepad2.right_trigger > 0.2) {
                        robot.setLiftPos(time, liftGrab, armDownFront, wristNeutral);
                        grabAdjust = 0;
                        grabbingBack = false;
                    }
                } else {
                    if (rbPressed) {
                        closeClaw = true;
                        robot.setLiftPos(time, grabAdjust, armDownFront, wristNeutral);
                        state = 1;
                    } else if (gamepad2.right_trigger > 0.2) {
                        robot.setLiftPos(time, liftGrab, armDownBack, wristNeutral);
                        grabAdjust = 0;
                        grabbingBack = true;
                    }
                }
            } else if (state == 1 && time > robot.restTime()) {
                if (closeClaw) {
                    robot.claw.setPosition(clawClosed);
                    if (rbReleased) {
                        if (grabbingBack) {
                            robot.setLiftPos(time + 0.25, grabHeight + grabAdjust, armDownBack, wristNeutral);
                        } else {
                            robot.setLiftPos(time + 0.25, grabHeight + grabAdjust, armDownFront, wristNeutral);
                        }
                        closeClaw = false;
                    }
                } else {
                    if (rbPressed) {
                        robot.setLiftPos(max(robot.restTime(), time), lastLiftX, lastArmX, lastWristX);
                        state = 2;
                    } else if (lbPressed) {
                        robot.claw.setPosition(clawOpen);
                        if (grabbingBack) {
                            robot.setLiftPos(max(robot.restTime(), time), liftGrab + grabAdjust, armDownBack, wristNeutral);
                        } else {
                            robot.setLiftPos(max(robot.restTime(), time), liftGrab + grabAdjust, armDownFront, wristNeutral);
                        }
                        state = 0;
                    } else if (gamepad2.left_trigger > 0.2) {
                        robot.setLiftPos(max(robot.restTime(), time), 0, armWait, wristNeutral);
                        waiting = true;
                        state = 2;
                    } else if (gamepad2.right_trigger > 0.2) {
                        if (aPressed) {
                            robot.setLiftPos(max(robot.restTime(), time), liftLow, armDropBack, wristDropBack);
                            state = 2;
                        } else if (bPressed) {
                            robot.setLiftPos(max(robot.restTime(), time), liftMid, armDropBack, wristDropBack);
                            state = 2;
                        } else if (yPressed) {
                            robot.setLiftPos(max(robot.restTime(), time), liftHigh, armDropBack, wristDropBack);
                            state = 2;
                        } else if (xPressed) {
                            robot.setLiftPos(max(robot.restTime(), time), liftGround, armGroundBack, wristNeutral);
                            state = 2;
                        }
                    } else {
                        if (aPressed) {
                            robot.setLiftPos(max(robot.restTime(), time), liftLow, armDropFront, wristDropFront);
                            state = 2;
                        } else if (bPressed) {
                            robot.setLiftPos(max(robot.restTime(), time), liftMid, armDropFront, wristDropFront);
                            state = 2;
                        } else if (yPressed) {
                            robot.setLiftPos(max(robot.restTime(), time), liftHigh, armDropFront, wristDropFront);
                            state = 2;
                        } else if (xPressed) {
                            robot.setLiftPos(max(robot.restTime(), time), liftGround, armGroundFront, wristNeutral);
                            state = 2;
                        }
                    }
                }
            } else if (state == 2 && time > robot.restTime()) {
                if (rbPressed && !waiting) {
                    lastLiftX = robot.liftProfile.getX(time);
                    lastArmX = robot.armProfile.getX(time);
                    lastWristX = robot.wristProfile.getX(time);
                    robot.claw.setPosition(clawOpen);
                    state = 3;
                } else if (gamepad2.left_trigger > 0.2) {
                    robot.setLiftPos(time, 0, armWait, wristNeutral);
                    waiting = true;
                } else if (gamepad2.right_trigger > 0.2) {
                    if (aPressed) {
                        robot.setLiftPos(time, liftLow, armDropBack, wristDropBack);
                        waiting = false;
                    } else if (bPressed) {
                        robot.setLiftPos(time, liftMid, armDropBack, wristDropBack);
                        waiting = false;
                    } else if (yPressed) {
                        robot.setLiftPos(time, liftHigh, armDropBack, wristDropBack);
                        waiting = false;
                    } else if (xPressed) {
                        robot.setLiftPos(time, liftGround, armGroundBack, wristNeutral);
                        waiting = false;
                    }
                } else {
                    if (aPressed) {
                        robot.setLiftPos(time, liftLow, armDropFront, wristDropFront);
                        waiting = false;
                    } else if (bPressed) {
                        robot.setLiftPos(time, liftMid, armDropFront, wristDropFront);
                        waiting = false;
                    } else if (yPressed) {
                        robot.setLiftPos(time, liftHigh, armDropFront, wristDropFront);
                        waiting = false;
                    } else if (xPressed) {
                        robot.setLiftPos(time, liftGround, armGroundFront, wristNeutral);
                        waiting = false;
                    }
                }
            } else if (state == 3) {
                if (gamepad2.right_trigger > 0.2 && rbPressed) {
                    robot.setLiftPos(time, liftGrab, armDownFront, wristNeutral);
                    grabbingBack = false;
                    grabAdjust = 0;
                    state = 0;
                } else if (rbPressed) {
                    robot.setLiftPos(time, liftGrab, armDownBack, wristNeutral);
                    grabbingBack = true;
                    grabAdjust = 0;
                    state = 0;
                }
            }
            if (gamepad2.dpad_up && time > robot.restTime()) {
                grabAdjust = min(grabAdjust + grabAdjustIncrement, grabAdjustMax);
                if (state == 0) {
                    robot.setLiftPos(time, liftGrab + grabAdjust, robot.armProfile.getX(time), wristNeutral);
                } else if (state == 1) {
                    robot.setLiftPos(time, grabHeight + grabAdjust, robot.armProfile.getX(time), wristNeutral);
                }
            } else if (gamepad2.dpad_down && time > robot.restTime()) {
                grabAdjust = max(grabAdjust - grabAdjustIncrement, 0);
                if (state == 0) {
                    robot.setLiftPos(time, liftGrab + grabAdjust, robot.armProfile.getX(time), wristNeutral);
                } else if (state == 1) {
                    robot.setLiftPos(time, grabHeight + grabAdjust, robot.armProfile.getX(time), wristNeutral);
                }
            }
            robot.update(time);
            moveAngle = atan2(-gamepad1.left_stick_x, -gamepad1.left_stick_y) - robot.getHeading() - initialHeading;
            moveMagnitude = gamepad1.left_stick_x * gamepad1.left_stick_x + gamepad1.left_stick_y * gamepad1.left_stick_y;
            turn = gamepad1.right_stick_x * abs(gamepad1.right_stick_x);
            if (moveMagnitude < 0.02) {
                moveMagnitude = 0;
            }
            if (abs(turn) < 0.02) {
                turn = 0;
            }
            if (gamepad1.right_trigger > 0.2) {
                robot.setDrivePowers(0.3 * (moveMagnitude * clip(sin(PI / 4 - moveAngle) / abs(cos(PI / 4 - moveAngle)), -1, 1) + turn),
                        0.3 * (moveMagnitude * clip(sin(PI / 4 + moveAngle) / abs(cos(PI / 4 + moveAngle)), -1, 1) - turn),
                        0.3 * (moveMagnitude * clip(sin(PI / 4 + moveAngle) / abs(cos(PI / 4 + moveAngle)),     -1, 1) + turn),
                        0.3 * (moveMagnitude * clip(sin(PI / 4 - moveAngle) / abs(cos(PI / 4 - moveAngle)), -1, 1) - turn));
            } else {
                robot.setDrivePowers(moveMagnitude * clip(sin(PI / 4 - moveAngle) / abs(cos(PI / 4 - moveAngle)), -1, 1) + turn,
                        moveMagnitude * clip(sin(PI / 4 + moveAngle) / abs(cos(PI / 4 + moveAngle)), -1, 1) - turn,
                        moveMagnitude * clip(sin(PI / 4 + moveAngle) / abs(cos(PI / 4 + moveAngle)), -1, 1) + turn,
                        moveMagnitude * clip(sin(PI / 4 - moveAngle) / abs(cos(PI / 4 - moveAngle)), -1, 1) - turn);
            }
        }
    }
}