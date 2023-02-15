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
@TeleOp(name = "TeleopOneDriver")
public class TeleopOneDriver extends LinearOpMode {
    Robot robot = new Robot();
    int state = 0;
    double initialHeading = ValueStorage.lastPose.getHeading() + PI / 2;
    double robotHeading;
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
        robot.setLiftPos(clock.seconds(), liftGrab, armDownBack, wristNeutral);
        robot.claw.setPosition(clawOpen);
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
                initialHeading -= robotHeading;
            }
            time = clock.seconds();
            if (state == 0 && time > robot.restTime()) {
                if (grabbingBack) {
                    if (rbPressed) {
                        closeClaw = true;
                        robot.setLiftPos(time, grabAdjust, armDownBack, wristNeutral);
                        state = 1;
                    } else if (gamepad1.right_trigger > 0.2) {
                        robot.setLiftPos(time, liftGrab, armDownFront, wristNeutral);
                        grabAdjust = 0;
                        grabbingBack = false;
                    }
                } else {
                    if (rbPressed) {
                        closeClaw = true;
                        robot.setLiftPos(time, grabAdjust, armDownFront, wristNeutral);
                        state = 1;
                    } else if (gamepad1.right_trigger > 0.2) {
                        robot.setLiftPos(time, liftGrab, armDownBack, wristNeutral);
                        grabAdjust = 0;
                        grabbingBack = true;
                    }
                }
            } else if (state == 1) {
                if (time > robot.restTime() && closeClaw) {
                    robot.claw.setPosition(clawClosed);
                    closeClaw = false;
                } else if (time > robot.restTime() + 0.25) {
                    if (rbPressed) {
                        robot.setLiftPos(time, lastLiftX, lastArmX, lastWristX);
                        state = 2;
                    } else if (lbPressed) {
                        robot.claw.setPosition(clawOpen);
                        if (grabbingBack) {
                            robot.setLiftPos(time + 0.25, liftGrab, armDownBack, wristNeutral);
                        } else {
                            robot.setLiftPos(time + 0.25, liftGrab, armDownFront, wristNeutral);
                        }
                        state = 1;
                    } else if (gamepad1.left_trigger > 0.2) {
                        robot.setLiftPos(time, 0, armWait, wristNeutral);
                        waiting = true;
                        state = 2;
                    } else if (gamepad1.right_trigger > 0.2) {
                        if (aPressed) {
                            robot.setLiftPos(time, liftLow, armDropBack, wristDropBack);
                            state = 2;
                        } else if (bPressed) {
                            robot.setLiftPos(time, liftMed, armDropBack, wristDropBack);
                            state = 2;
                        } else if (yPressed) {
                            robot.setLiftPos(time, liftHigh, armDropBack, wristDropBack);
                            state = 2;
                        } else if (xPressed) {
                            robot.setLiftPos(time, liftGround, armDownBack, wristNeutral);
                            state = 2;
                        }
                    } else {
                        if (aPressed) {
                            robot.setLiftPos(time, liftLow, armDropFront, wristDropFront);
                            state = 2;
                        } else if (bPressed) {
                            robot.setLiftPos(time, liftMed, armDropFront, wristDropFront);
                            state = 2;
                        } else if (yPressed) {
                            robot.setLiftPos(time, liftHigh, armDropFront, wristDropFront);
                            state = 2;
                        } else if (xPressed) {
                            robot.setLiftPos(time, liftGround, armDownFront, wristNeutral);
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
                } else if (gamepad1.left_trigger > 0.2) {
                    robot.setLiftPos(time, 0, armWait, wristNeutral);
                    waiting = true;
                } else if (gamepad1.right_trigger > 0.2) {
                    if (aPressed) {
                        robot.setLiftPos(time, liftLow, armDropBack, wristDropBack);
                        waiting = false;
                    } else if (bPressed) {
                        robot.setLiftPos(time, liftMed, armDropBack, wristDropBack);
                        waiting = false;
                    } else if (yPressed) {
                        robot.setLiftPos(time, liftHigh, armDropBack, wristDropBack);
                        waiting = false;
                    } else if (xPressed) {
                        robot.setLiftPos(time, liftGround, armDownBack, wristNeutral);
                        waiting = false;
                    }
                } else {
                    if (aPressed) {
                        robot.setLiftPos(time, liftLow, armDropFront, wristDropFront);
                        waiting = false;
                    } else if (bPressed) {
                        robot.setLiftPos(time, liftMed, armDropFront, wristDropFront);
                        waiting = false;
                    } else if (yPressed) {
                        robot.setLiftPos(time, liftHigh, armDropFront, wristDropFront);
                        waiting = false;
                    } else if (xPressed) {
                        robot.setLiftPos(time, liftGround, armDownFront, wristNeutral);
                        waiting = false;
                    }
                }
            } else if (state == 3 && time > robot.restTime() + 0.25) {
                if (gamepad1.right_trigger > 0.2 && rbPressed) {
                    robot.setLiftPos(time, liftGrab, armDownFront, wristNeutral);
                    grabbingBack = false;
                    state = 0;
                } else if (rbPressed) {
                    robot.setLiftPos(time, liftGrab, armDownBack, wristNeutral);
                    grabbingBack = true;
                    grabAdjust = 0;
                    state = 0;
                }
            }
            if (gamepad1.dpad_up) {
                if (time > robot.restTime()) {
                    grabAdjust = min(grabAdjust + grabAdjustIncrement, grabAdjustMax);
                    if (state == 0 && time > robot.restTime()) {
                        robot.setLiftPos(time, liftGrab + grabAdjust, robot.armProfile.getX(time), wristNeutral);
                    } else if (state == 1) {
                        robot.setLiftPos(time, grabAdjust, robot.armProfile.getX(time), wristNeutral);
                    }
                }
            } else if (gamepad1.dpad_down) {
                if (time > robot.restTime()) {
                    grabAdjust = max(grabAdjust - grabAdjustIncrement, 0);
                    if (state == 0 && time > robot.restTime()) {
                        robot.setLiftPos(time, liftGrab + grabAdjust, robot.armProfile.getX(time), wristNeutral);
                    } else if (state == 1) {
                        robot.setLiftPos(time, grabAdjust, robot.armProfile.getX(time), wristNeutral);
                    }
                }
            }
            robot.update(time);
            robotHeading = robot.heading() + initialHeading;
            moveAngle = atan2(-gamepad1.left_stick_x, -gamepad1.left_stick_y) - robotHeading;
            moveMagnitude = pow(pow(gamepad1.left_stick_x, 2) + pow(gamepad1.left_stick_y, 2), 1.5);
            if (moveMagnitude < 0.01) {
                moveMagnitude = 0;
            }
            turn = pow(gamepad1.right_stick_x, 3);
            robot.setDrivePowers(moveMagnitude * clip(sin(PI / 4 - moveAngle) / abs(cos(PI / 4 - moveAngle)), -1, 1) + turn,
                    moveMagnitude * clip(sin(PI / 4 + moveAngle) / abs(cos(PI / 4 + moveAngle)), -1, 1) - turn,
                    moveMagnitude * clip(sin(PI / 4 + moveAngle) / abs(cos(PI / 4 + moveAngle)), -1, 1) + turn,
                    moveMagnitude * clip(sin(PI / 4 - moveAngle) / abs(cos(PI / 4 - moveAngle)), -1, 1) - turn);
        }
    }
}