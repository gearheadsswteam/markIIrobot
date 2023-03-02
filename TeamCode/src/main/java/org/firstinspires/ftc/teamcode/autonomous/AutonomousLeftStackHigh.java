package org.firstinspires.ftc.teamcode.autonomous;
import static org.firstinspires.ftc.teamcode.classes.ValueStorage.*;
import static java.lang.Math.*;
import com.acmerobotics.roadrunner.geometry.Pose2d;
import com.acmerobotics.roadrunner.geometry.Vector2d;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import org.firstinspires.ftc.teamcode.drive.DriveConstants;
import org.firstinspires.ftc.teamcode.drive.SampleMecanumDrive;
import org.firstinspires.ftc.teamcode.trajectorysequence.TrajectorySequence;
@Autonomous(name = "LeftStackHigh")
public class AutonomousLeftStackHigh extends AbstractAutonomous {
    Pose2d dropPose1 = new Pose2d(27, 5, PI + 1);
    Pose2d dropPose2 = new Pose2d(29, 3, PI + 0.6);
    Pose2d stackPose = new Pose2d(67, 13, PI);
    Pose2d[] parkPose = new Pose2d[] {new Pose2d(60, 13, PI), new Pose2d(36, 13, PI), new Pose2d(12, 13, PI)};
    TrajectorySequence traj1;
    TrajectorySequence traj2;
    TrajectorySequence traj3;
    TrajectorySequence[] traj4;
    double[] stackOffsets = {320, 240, 160, 80, 0};
    int totalCycles = 5;
    int cycles  = 0;
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
                    robot.setLiftPos(time, liftHigh, armDropFront, wristDropFront);
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
                    robot.setLiftPos(time, stackOffsets[0], armDownBack, wristNeutral);
                })
                .UNSTABLE_addTemporalMarkerOffset(0.15, () -> {
                    robot.claw.setPosition(clawClosed);
                    robot.setLiftPos(time + 0.25, stackOffsets[0] + 300, armDownBack, wristNeutral);
                })
                .waitSeconds(0.6)
                .lineTo(new Vector2d(57, 13))
                .setAccelConstraint(SampleMecanumDrive.getAccelerationConstraint(40))
                .splineTo(dropPose2.vec(), dropPose2.getHeading())
                .addTemporalMarker(0, 0, () -> {
                    robot.setLiftPos(time, liftGrab + stackOffsets[0], armDownBack, wristNeutral);
                })
                .addTemporalMarker(1, -2,() -> {
                    robot.setLiftPos(time, liftHigh, armDropFront, wristDropFront);
                })
                .addTemporalMarker(1, 0, () -> {
                    robot.claw.setPosition(clawOpen);
                    cycles++;
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
                    robot.setLiftPos(time, stackOffsets[cycles], armDownBack, wristNeutral);
                })
                .UNSTABLE_addTemporalMarkerOffset(0.15, () -> {
                    robot.claw.setPosition(clawClosed);
                    robot.setLiftPos(time + 0.25, stackOffsets[0] + 300, armDownBack, wristNeutral);
                })
                .waitSeconds(0.6)
                .setReversed(false)
                .lineTo(new Vector2d(57, 13))
                .setAccelConstraint(SampleMecanumDrive.getAccelerationConstraint(40))
                .splineTo(dropPose2.vec(), dropPose2.getHeading())
                .addTemporalMarker(1, -2,() -> {
                    robot.setLiftPos(time, liftHigh, armDropFront, wristDropFront);
                })
                .addTemporalMarker(0, 0, () -> {
                    robot.setLiftPos(time, liftGrab + stackOffsets[cycles], armDownBack, wristNeutral);
                })
                .addTemporalMarker(1, 0, () -> {
                    robot.claw.setPosition(clawOpen);
                    cycles++;
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
                        .addTemporalMarker(0, 0, () -> {
                            robot.setLiftPos(time, 0, armWait, wristNeutral);
                            readyToEnd = true;
                        })
                        .addTemporalMarker(1, 0, () -> {
                            parkDone = true;
                        })
                        .build(),
                robot.drive.trajectorySequenceBuilder(dropPose2)
                        .setVelConstraint(SampleMecanumDrive.getVelocityConstraint(40, DriveConstants.MAX_ANG_VEL, DriveConstants.TRACK_WIDTH))
                        .setAccelConstraint(SampleMecanumDrive.getAccelerationConstraint(40))
                        .lineToLinearHeading(parkPose[1])
                        .addTemporalMarker(0, 0, () -> {
                            robot.setLiftPos(time, 0, armWait, wristNeutral);
                            readyToEnd = true;
                        })
                        .addTemporalMarker(1, 0, () -> {
                            parkDone = true;
                        })
                        .build(),
                robot.drive.trajectorySequenceBuilder(dropPose2)
                        .lineToLinearHeading(parkPose[1])
                        .lineTo(parkPose[2].vec())
                        .addTemporalMarker(0, 0, () -> {
                            robot.setLiftPos(time, 0, armWait, wristNeutral);
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
        while(opModeIsActive() && !isStopRequested() && (!parkDone || (!readyToEnd && time < robot.restTime()))) {
            time = clock.seconds();
            robot.drive.update();
            robot.update(time);
        }
    }
    @Override
    public Pose2d initPose() {
        return new Pose2d(32, 64, -PI / 2);
    }
}
