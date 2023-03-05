package org.firstinspires.ftc.teamcode.autonomous;
import static org.firstinspires.ftc.teamcode.classes.ValueStorage.*;
import static java.lang.Math.*;
import com.acmerobotics.roadrunner.geometry.Pose2d;
import com.acmerobotics.roadrunner.geometry.Vector2d;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.Disabled;
import org.firstinspires.ftc.teamcode.drive.DriveConstants;
import org.firstinspires.ftc.teamcode.drive.SampleMecanumDrive;
import org.firstinspires.ftc.teamcode.trajectorysequence.TrajectorySequence;
@Autonomous(name = "LeftStackHigh")
@Disabled
public class AutonomousLeftStackHigh extends AbstractAutonomous {
    Pose2d dropPose1 = new Pose2d(27, 5, PI + 1);
    Pose2d dropPose2 = new Pose2d(29, 3, PI + 0.6);
    Pose2d stackPose = new Pose2d(67, 13, PI);
    Pose2d[] parkPose = new Pose2d[] {new Pose2d(62, 11, PI), new Pose2d(38, 11, PI), new Pose2d(14, 11, PI)};
    TrajectorySequence traj1;
    TrajectorySequence traj2;
    TrajectorySequence traj3;
    TrajectorySequence[] traj4;
    double[] stackOffsets = {400, 300, 200, 100, 0};
    double[] servoCurrent = new double[50];
    double averageCurrent;
    int totalCycles = 5;
    int grabCycles = 0;
    int cycles = 0;
    boolean readyToEnd;
    boolean parkDone;
    @Override
    public void initialize() {
        traj1 = robot.drive.trajectorySequenceBuilder(initPose())
                .setVelConstraint(SampleMecanumDrive.getVelocityConstraint(55, DriveConstants.MAX_ANG_VEL, DriveConstants.TRACK_WIDTH))
                .setAccelConstraint(SampleMecanumDrive.getAccelerationConstraint(60))
                .splineTo(new Vector2d(35, 35), -PI / 2)
                .setAccelConstraint(SampleMecanumDrive.getAccelerationConstraint(40))
                .splineTo(dropPose1.vec(), dropPose1.getHeading())
                .addTemporalMarker(1, -1.75, () -> {
                    robot.setLiftPos(time, liftHigh, armDropFront);
                })
                .addTemporalMarker(1,   0, () -> {
                    robot.claw.setPosition(clawOpen);
                    robot.drive.followTrajectorySequenceAsync(traj2);
                })
                .build();
        traj2 = robot.drive.trajectorySequenceBuilder(dropPose1)
                .setVelConstraint(SampleMecanumDrive.getVelocityConstraint(55, DriveConstants.MAX_ANG_VEL, DriveConstants.TRACK_WIDTH))
                .setAccelConstraint(SampleMecanumDrive.getAccelerationConstraint(60))
                .setTangent(0.6)
                .splineToSplineHeading(new Pose2d(57, 13, PI), 0)
                .lineTo(stackPose.vec())
                .UNSTABLE_addTemporalMarkerOffset(0, () -> {
                    robot.setLiftPos(time, stackOffsets[0], armDownBack);
                })
                .UNSTABLE_addTemporalMarkerOffset(0.1, () -> {
                    robot.claw.setPosition(clawClosed);
                })
                .UNSTABLE_addTemporalMarkerOffset(0.3, () -> {
                    robot.setLiftPos(time, liftMid, armDownBack);
                })
                .waitSeconds(0.6)
                .lineTo(new Vector2d(57, 13))
                .setAccelConstraint(SampleMecanumDrive.getAccelerationConstraint(40))
                .splineTo(dropPose2.vec(), dropPose2.getHeading())
                .addTemporalMarker(0, 0.25, () -> {
                    robot.setLiftPos(time, liftGrab + stackOffsets[0], armDownBack);
                })
                .addTemporalMarker(1, -1.5,() -> {
                    robot.setLiftPos(time, liftHigh, armDropFront);
                })
                .addTemporalMarker(1, 0, () -> {
                    robot.claw.setPosition(clawOpen);
                    cycles++;
                    if (averageCurrent > currentThreshold) {
                        grabCycles++;
                    }
                    if (cycles < totalCycles) {
                        robot.drive.followTrajectorySequenceAsync(traj3);
                    } else {
                        robot.drive.followTrajectorySequenceAsync(traj4[runCase - 1]);
                    }
                })
                .build();
        traj3 = robot.drive.trajectorySequenceBuilder(dropPose2)
                .setVelConstraint(SampleMecanumDrive.getVelocityConstraint(55, DriveConstants.MAX_ANG_VEL, DriveConstants.TRACK_WIDTH))
                .setAccelConstraint(SampleMecanumDrive.getAccelerationConstraint(60))
                .setReversed(true)
                .splineTo(new Vector2d(57, 13), 0)
                .lineTo(stackPose.vec())
                .UNSTABLE_addTemporalMarkerOffset(0, () -> {
                    robot.setLiftPos(time, stackOffsets[grabCycles], armDownBack);
                })
                .UNSTABLE_addTemporalMarkerOffset(0.1, () -> {
                    robot.claw.setPosition(clawClosed);
                })
                .UNSTABLE_addTemporalMarkerOffset(0.3, () -> {
                    robot.setLiftPos(time, liftMid, armDownBack);
                })
                .waitSeconds(0.5)
                .setReversed(false)
                .lineTo(new Vector2d(57, 13))
                .setAccelConstraint(SampleMecanumDrive.getAccelerationConstraint(40))
                .splineTo(dropPose2.vec(), dropPose2.getHeading())
                .addTemporalMarker(1, -1.5,() -> {
                    robot.setLiftPos(time, liftHigh, armDropFront);
                })
                .addTemporalMarker(0, 0.25, () -> {
                    robot.setLiftPos(time, liftGrab + stackOffsets[grabCycles], armDownBack);
                })
                .addTemporalMarker(1, 0, () -> {
                    robot.claw.setPosition(clawOpen);
                    cycles++;
                    if (averageCurrent > currentThreshold) {
                        grabCycles++;
                    }
                    if (cycles < totalCycles) {
                        robot.drive.followTrajectorySequenceAsync(traj3);
                    } else {
                        robot.drive.followTrajectorySequenceAsync(traj4[runCase - 1]);
                    }
                })
                .build();
        traj4 = new TrajectorySequence[] {
                robot.drive.trajectorySequenceBuilder(dropPose2)
                        .setReversed(true)
                        .splineTo(parkPose[0].vec(), 0)
                        .addTemporalMarker(0, 0.25, () -> {
                            robot.setLiftPos(time, 0, armWait);
                            readyToEnd = true;
                        })
                        .addTemporalMarker(1, 0, () -> {
                            parkDone = true;
                        })
                        .build(),
                robot.drive.trajectorySequenceBuilder(dropPose2)
                        .setVelConstraint(SampleMecanumDrive.getVelocityConstraint(40, DriveConstants.MAX_ANG_VEL, DriveConstants.TRACK_WIDTH))
                        .setAccelConstraint(SampleMecanumDrive.getAccelerationConstraint(40))
                        .back(8)
                        .splineToSplineHeading(parkPose[1], PI / 2)
                        .addTemporalMarker(0, 0.25, () -> {
                            robot.setLiftPos(time, 0, armWait);
                            readyToEnd = true;
                        })
                        .addTemporalMarker(1, 0, () -> {
                            parkDone = true;
                        })
                        .build(),
                robot.drive.trajectorySequenceBuilder(dropPose2)
                        .back(8)
                        .splineToSplineHeading(parkPose[1], PI / 2)
                        .lineTo(parkPose[0].vec())
                        .addTemporalMarker(0, 0.25, () -> {
                            robot.setLiftPos(time, 0, armWait);
                            readyToEnd = true;
                        })
                        .addTemporalMarker(1, 0, () -> {
                            parkDone = true;
                        })
                        .build()};
    }
    @Override
    public void run() {
        robot.drive.followTrajectorySequenceAsync(traj1);
        while(opModeIsActive() && !isStopRequested() && (!parkDone || (!readyToEnd && time < robot.restTime() + 0.25))) {
            time = clock.seconds();
            for (int i = servoCurrent.length - 1; i >= 0 ; i--) {
                if (i == servoCurrent.length - 1) {
                    averageCurrent -= servoCurrent[i] / servoCurrent.length;
                }
                if (i > 0) {
                    servoCurrent[i] = servoCurrent[i - 1];
                } else {
                    servoCurrent[0] = robot.servoCurrent();
                    averageCurrent += servoCurrent[0] / servoCurrent.length;
                }
            }
            robot.drive.update();
            robot.update(time);
        }
    }
    @Override
    public Pose2d initPose() {
        return new Pose2d(32, 64, -PI / 2);
    }
}
