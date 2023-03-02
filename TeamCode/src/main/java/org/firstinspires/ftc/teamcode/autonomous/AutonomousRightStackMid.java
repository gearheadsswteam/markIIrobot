package org.firstinspires.ftc.teamcode.autonomous;
import static org.firstinspires.ftc.teamcode.classes.ValueStorage.*;
import static java.lang.Math.*;
import com.acmerobotics.roadrunner.geometry.Pose2d;
import com.acmerobotics.roadrunner.geometry.Vector2d;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import org.firstinspires.ftc.teamcode.drive.DriveConstants;
import org.firstinspires.ftc.teamcode.drive.SampleMecanumDrive;
import org.firstinspires.ftc.teamcode.trajectorysequence.TrajectorySequence;
@Autonomous(name = "RightStackMid")
public class AutonomousRightStackMid extends AbstractAutonomous {
    Pose2d dropPose = new Pose2d(-29, 22, 0.6);
    Pose2d stackPose = new Pose2d(-66, 13, 0);
    Pose2d[] parkPose = new Pose2d[] {new Pose2d(-12, 13, 0), new Pose2d(-36, 13, 0), new Pose2d(-60, 13, 0)};
    TrajectorySequence traj1;
    TrajectorySequence traj2;
    TrajectorySequence[] traj3;
    double[] stackOffsets = {320, 240, 160, 80, 0};
    int totalCycles = 5;
    int cycles = 0;
    boolean readyToEnd;
    boolean parkDone;
    @Override
    public void initialize() {
        traj1 = robot.drive.trajectorySequenceBuilder(initPose())
                .setVelConstraint(SampleMecanumDrive.getVelocityConstraint(55, DriveConstants.MAX_ANG_VEL, DriveConstants.TRACK_WIDTH))
                .setAccelConstraint(SampleMecanumDrive.getAccelerationConstraint(60))
                .splineTo(new Vector2d(-35, 35), -PI / 2)
                .lineToSplineHeading(new Pose2d(-35, 18, 0.6))
                .lineTo(dropPose.vec())
                .addTemporalMarker(1, -1.5, () -> {
                    robot.setLiftPos(time, liftMid, armDropFront, wristDropFront);
                })
                .addTemporalMarker(1,   0, () -> {
                    robot.claw.setPosition(clawOpen);
                    robot.drive.followTrajectorySequenceAsync(traj2);
                })
                .build();
        traj2 = robot.drive.trajectorySequenceBuilder(dropPose)
                .setVelConstraint(SampleMecanumDrive.getVelocityConstraint(55, DriveConstants.MAX_ANG_VEL, DriveConstants.TRACK_WIDTH))
                .setAccelConstraint(SampleMecanumDrive.getAccelerationConstraint(60))
                .setReversed(true)
                .splineTo(new Vector2d(-55, 13), PI)
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
                .lineTo(new Vector2d(-55, 13))
                .setAccelConstraint(SampleMecanumDrive.getAccelerationConstraint(50))
                .splineTo(dropPose.vec(), dropPose.getHeading())
                .addTemporalMarker(1, -1.5,() -> {
                    robot.setLiftPos(time, liftMid, armDropFront, wristDropFront);
                })
                .addTemporalMarker(0, 0, () -> {
                    robot.setLiftPos(time, liftGrab + stackOffsets[cycles], armDownBack, wristNeutral);
                })
                .addTemporalMarker(1, 0, () -> {
                    robot.claw.setPosition(clawOpen);
                    cycles++;
                    if (cycles < totalCycles) {
                        robot.drive.followTrajectorySequenceAsync(traj2);
                    } else {
                        robot.drive.followTrajectorySequenceAsync(traj3[runCase - 1]);
                    }
                })
                .build();
        traj3 = new TrajectorySequence[] {
                robot.drive.trajectorySequenceBuilder(dropPose)
                        .lineToLinearHeading(parkPose[1])
                        .lineTo(parkPose[0].vec())
                        .addTemporalMarker(0, 0, () -> {
                            robot.setLiftPos(time, 0, armWait, wristNeutral);
                            readyToEnd = true;
                        })
                        .addTemporalMarker(1, 0, () -> {
                            parkDone = true;
                        })
                        .build(),
                robot.drive.trajectorySequenceBuilder(dropPose)
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
                robot.drive.trajectorySequenceBuilder(dropPose)
                        .setReversed(true)
                        .splineTo(parkPose[2].vec(), PI)
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
        return new Pose2d(-32, 64, -PI / 2);
    }
}
