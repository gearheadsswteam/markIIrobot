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
@TeleOp(name = "OneDriver")
public class TeleopOneDriver extends LinearOpMode {
    Robot robot = new Robot();
    int state = 0;
    double initialHeading = ValueStorage.lastPose.getHeading() + PI / 2;
    double moveAngle;
    double moveMagnitude;
    double turn;
    double time;
    double restTime;
    double lastLiftX = liftHigh;
    double lastArmX = armDropFront;
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
        robot.init(hardwareMap, 0, armWait, true);
        robot.startGyro(this);
        robot.setLiftPos(clock.seconds(), liftGrab, armDownBack);
        while (!isStarted() && !isStopRequested()) {
            robot.update(clock.seconds());
        }
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
            if (gamepad1.ps) {
                initialHeading = -robot.getHeading();
            }
            time = clock.seconds();
            if (state == 0 && time > robot.restTime()) {
                if (grabbingBack) {
                    if (rbPressed) {
                        closeClaw = true;
                        robot.setLiftPos(time, grabAdjust, armDownBack);
                        state = 1;
                    } else if (gamepad1.right_trigger > 0.2) {
                        robot.setLiftPos(time, liftGrab, armDownFront);
                        grabAdjust = 0;
                        grabbingBack = false;
                    }
                } else {
                    if (rbPressed) {
                        closeClaw = true;
                        robot.setLiftPos(time, grabAdjust, armDownFront);
                        state = 1;
                    } else if (gamepad1.right_trigger > 0.2) {
                        robot.setLiftPos(time, liftGrab, armDownBack);
                        grabAdjust = 0;
                        grabbingBack = true;
                    }
                }
            } else if (state == 1 && time > robot.restTime()) {
                if (closeClaw) {
                    robot.claw.setPosition(clawClosed);
                    if (rbReleased) {
                        if (grabbingBack) {
                            robot.setLiftPos(time + 0.25, grabHeight + grabAdjust, armDownBack);
                        } else {
                            robot.setLiftPos(time + 0.25, grabHeight + grabAdjust, armDownFront);
                        }
                        closeClaw = false;
                    }
                } else {
                    if (rbPressed) {
                        robot.setLiftPos(max(robot.restTime(), time), lastLiftX, lastArmX);
                        restTime = robot.restTime();
                        state = 2;
                    } else if (lbPressed) {
                        robot.claw.setPosition(clawOpen);
                        if (grabbingBack) {
                            robot.setLiftPos(max(robot.restTime(), time), liftGrab + grabAdjust, armDownBack);
                        } else {
                            robot.setLiftPos(max(robot.restTime(), time), liftGrab + grabAdjust, armDownFront);
                        }
                        state = 0;
                    } else if (gamepad1.left_trigger > 0.2) {
                        robot.setLiftPos(max(robot.restTime(), time), 0, armWait);
                        waiting = true;
                        state = 2;
                    } else if (gamepad1.right_trigger > 0.2) {
                        if (aPressed) {
                            robot.setLiftPos(max(robot.restTime(), time), liftLow, armDropBack);
                            restTime = robot.restTime();
                            state = 2;
                        } else if (bPressed) {
                            robot.setLiftPos(max(robot.restTime(), time), liftMid, armDropBack);
                            restTime = robot.restTime();
                            state = 2;
                        } else if (yPressed) {
                            robot.setLiftPos(max(robot.restTime(), time), liftHigh, armDropBack);
                            restTime = robot.restTime();
                            state = 2;
                        } else if (xPressed) {
                            robot.setLiftPos(max(robot.restTime(), time), liftGround, armGroundBack);
                            restTime = robot.restTime();
                            state = 2;
                        }
                    } else {
                        if (aPressed) {
                            robot.setLiftPos(max(robot.restTime(), time), liftLow, armDropFront);
                            restTime = robot.restTime();
                            state = 2;
                        } else if (bPressed) {
                            robot.setLiftPos(max(robot.restTime(), time), liftMid, armDropFront);
                            restTime = robot.restTime();
                            state = 2;
                        } else if (yPressed) {
                            robot.setLiftPos(max(robot.restTime(), time), liftHigh, armDropFront);
                            restTime = robot.restTime();
                            state = 2;
                        } else if (xPressed) {
                            robot.setLiftPos(max(robot.restTime(), time), liftGround, armGroundFront);
                            restTime = robot.restTime();
                            state = 2;
                        }
                    }
                }
            } else if (state == 2 && time > robot.restTime()) {
                if (rbPressed && !waiting) {
                    lastLiftX = robot.liftProfile.getX(time);
                    lastArmX = robot.armProfile.getX(time);
                    robot.claw.setPosition(clawOpen);
                    state = 3;
                } else if (gamepad1.left_trigger > 0.2) {
                    robot.setLiftPos(time, 0, armWait);
                    waiting = true;
                } else if (gamepad1.right_trigger > 0.2) {
                    if (aPressed) {
                        robot.setLiftPos(time, liftLow, armDropBack);
                        restTime = robot.restTime();
                        waiting = false;
                    } else if (bPressed) {
                        robot.setLiftPos(time, liftMid, armDropBack);
                        restTime = robot.restTime();
                        waiting = false;
                    } else if (yPressed) {
                        robot.setLiftPos(time, liftHigh, armDropBack);
                        restTime = robot.restTime();
                        waiting = false;
                    } else if (xPressed) {
                        robot.setLiftPos(time, liftGround, armGroundBack);
                        restTime = robot.restTime();
                        waiting = false;
                    }
                } else {
                    if (aPressed) {
                        robot.setLiftPos(time, liftLow, armDropFront);
                        restTime = robot.restTime();
                        waiting = false;
                    } else if (bPressed) {
                        robot.setLiftPos(time, liftMid, armDropFront);
                        restTime = robot.restTime();
                        waiting = false;
                    } else if (yPressed) {
                        robot.setLiftPos(time, liftHigh, armDropFront);
                        restTime = robot.restTime();
                        waiting = false;
                    } else if (xPressed) {
                        robot.setLiftPos(time, liftGround, armGroundFront);
                        restTime = robot.restTime();
                        waiting = false;
                    }
                }
            } else if (state == 3) {
                if (gamepad1.right_trigger > 0.2 && rbPressed) {
                    robot.setLiftPos(time, liftGrab, armDownFront);
                    grabbingBack = false;
                    grabAdjust = 0;
                    state = 0;
                } else if (rbPressed) {
                    robot.setLiftPos(time, liftGrab, armDownBack);
                    grabbingBack = true;
                    grabAdjust = 0;
                    state = 0;
                }
            }
            if (gamepad1.dpad_up) {
                if (state == 0 && time > robot.restTime() && !closeClaw) {
                    grabAdjust = min(grabAdjust + grabAdjustIncrement, grabAdjustMax);
                    robot.setLiftPos(time, liftGrab + grabAdjust, robot.armProfile.getX(time));
                } else if (state == 1 && time > robot.restTime() && !closeClaw) {
                    grabAdjust = min(grabAdjust + grabAdjustIncrement, grabAdjustMax);
                    robot.setLiftPos(time, grabHeight + grabAdjust, robot.armProfile.getX(time));
                } else if ((state == 2 || state == 3) && time > restTime && !waiting) {
                    double liftAdjust = min(robot.liftProfile.getX(time) + liftAdjustIncrement, liftAdjustMax);
                    if (robot.armProfile.getX(time) < armWait) {
                        robot.setLiftPos(time, liftAdjust, armAdjustBack(liftAdjust));
                    } else {
                        robot.setLiftPos(time, liftAdjust, armAdjustFront(liftAdjust));
                    }
                }
            } else if (gamepad1.dpad_down) {
                if (state == 0 && time > robot.restTime() && !closeClaw) {
                    grabAdjust = max(grabAdjust - grabAdjustIncrement, 0);
                    robot.setLiftPos(time, liftGrab + grabAdjust, robot.armProfile.getX(time));
                } else if (state == 1 && time > robot.restTime() && !closeClaw) {
                    grabAdjust = max(grabAdjust - grabAdjustIncrement, 0);
                    robot.setLiftPos(time, grabHeight + grabAdjust, robot.armProfile.getX(time));
                } else if ((state == 2 || state == 3) && time > restTime && !waiting) {
                    double liftAdjust = max(robot.liftProfile.getX(time) - liftAdjustIncrement, liftGround);
                    if (robot.armProfile.getX(time) < armWait) {
                        robot.setLiftPos(time, liftAdjust, armAdjustBack(liftAdjust));
                    } else {
                        robot.setLiftPos(time, liftAdjust, armAdjustFront(liftAdjust));
                    }
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
            robot.setDrivePowers(moveMagnitude * clip(sin(PI / 4 - moveAngle) / abs(cos(PI / 4 - moveAngle)), -1, 1) + turn,
                    moveMagnitude * clip(sin(PI / 4 + moveAngle) / abs(cos(PI / 4 + moveAngle)), -1, 1) - turn,
                    moveMagnitude * clip(sin(PI / 4 + moveAngle) / abs(cos(PI / 4 + moveAngle)),     -1, 1) + turn,
                    moveMagnitude * clip(sin(PI / 4 - moveAngle) / abs(cos(PI / 4 - moveAngle)), -1, 1) - turn);
        }
    }
}