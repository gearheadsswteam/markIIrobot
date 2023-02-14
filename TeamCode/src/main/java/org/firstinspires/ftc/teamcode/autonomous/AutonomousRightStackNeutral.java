package org.firstinspires.ftc.teamcode.autonomous;
import static org.firstinspires.ftc.teamcode.classes.ValueStorage.*;
import static java.lang.Math.*;
import com.acmerobotics.roadrunner.geometry.Pose2d;
import com.acmerobotics.roadrunner.geometry.Vector2d;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import org.firstinspires.ftc.teamcode.drive.DriveConstants;
import org.firstinspires.ftc.teamcode.drive.SampleMecanumDrive;
import org.firstinspires.ftc.teamcode.trajectorysequence.TrajectorySequence;
@Autonomous(name = "RightStackNeutral")
public class AutonomousRightStackNeutral extends AbstractAutonomous {
    Pose2d dropPose1 = new Pose2d(-27, 6, -1);
    Pose2d dropPose2 = new Pose2d(-29, 4, -0.5);
    Pose2d stackPose = new Pose2d(-66, 13, 0);
    Pose2d[] parkPose = new Pose2d[] {new Pose2d(-11, 11, 0), new Pose2d(-35, 11, 0), new Pose2d(-59, 11, 0)};
    TrajectorySequence traj1;
    TrajectorySequence traj2;
    TrajectorySequence traj3;
    TrajectorySequence[] traj4;
    double[] stackOffsets = {400, 300, 200, 100, 0};
    int totalCycles = 5;
    int cycles  = 0;
    boolean readyToEnd;
    @Override
    public void initialize() {
        traj1 = robot.drive.trajectorySequenceBuilder(initPose())
                .setVelConstraint(SampleMecanumDrive.getVelocityConstraint(45, DriveConstants.MAX_ANG_VEL, DriveConstants.TRACK_WIDTH))
                .setAccelConstraint(SampleMecanumDrive.getAccelerationConstraint(45))
                .splineTo(new Vector2d(-34, 45), -PI / 2)
                .splineTo(dropPose1.vec(), dropPose1.getHeading())
                .waitSeconds(0.25)
                .resetVelConstraint()
                .resetAccelConstraint()
                .addTemporalMarker(1, -2, () -> {
                    robot.setLiftPos(time, liftHigh, armDropFront, wristDropFront);
                })
                .addTemporalMarker(1, -0.25, () -> {
                    robot.claw.setPosition(clawOpen);
                })
                .addTemporalMarker(1, 0, () -> {
                    robot.drive.followTrajectorySequenceAsync(traj2);
                })
                .build();
        traj2 = robot.drive.trajectorySequenceBuilder(dropPose1)
                .setVelConstraint(SampleMecanumDrive.getVelocityConstraint(50, DriveConstants.MAX_ANG_VEL, DriveConstants.TRACK_WIDTH))
                .setAccelConstraint(SampleMecanumDrive.getAccelerationConstraint(50))
                .setReversed(true)
                .splineTo(stackPose.vec(), stackPose.getHeading() + PI)
                .UNSTABLE_addTemporalMarkerOffset(0, () -> {
                    robot.setLiftPos(time, stackOffsets[0], armDownBack, wristNeutral);
                })
                .UNSTABLE_addTemporalMarkerOffset(0.25, () -> {
                    robot.claw.setPosition(clawClosed);
                    robot.setLiftPos(time + 0.25, stackOffsets[0] + 500, armDownBack, wristNeutral);
                })
                .waitSeconds(1)
                .setReversed(false)
                .setVelConstraint(SampleMecanumDrive.getVelocityConstraint(40, DriveConstants.MAX_ANG_VEL, DriveConstants.TRACK_WIDTH))
                .setAccelConstraint(SampleMecanumDrive.getAccelerationConstraint(40))
                .splineTo(dropPose2.vec(), dropPose2.getHeading())
                .waitSeconds(0.25)
                .addTemporalMarker(1, -2,() -> {
                    robot.setLiftPos(time, liftHigh, armDropFront, wristDropFront);
                })
                .addTemporalMarker(0, 0, () -> {
                    robot.setLiftPos(time, liftGrab + stackOffsets[0], armDownBack, wristNeutral);
                })
                .addTemporalMarker(1, -0.25, () -> {
                    robot.claw.setPosition(clawOpen);
                })
                .addTemporalMarker(1, 0, () -> {
                    cycles++;
                    if (cycles < totalCycles) {
                        robot.drive.followTrajectorySequenceAsync(traj3);
                    } else {
                        robot.drive.followTrajectorySequenceAsync(traj4[runCase]);
                    }
                })
                .build();
        traj3 = robot.drive.trajectorySequenceBuilder(dropPose2)
                .setVelConstraint(SampleMecanumDrive.getVelocityConstraint(50, DriveConstants.MAX_ANG_VEL, DriveConstants.TRACK_WIDTH))
                .setAccelConstraint(SampleMecanumDrive.getAccelerationConstraint(50))
                .setReversed(true)
                .splineTo(stackPose.vec(), stackPose.getHeading() + PI)
                .UNSTABLE_addTemporalMarkerOffset(0, () -> {
                    robot.setLiftPos(time, stackOffsets[cycles], armDownBack, wristNeutral);
                })
                .UNSTABLE_addTemporalMarkerOffset(0.25, () -> {
                    robot.claw.setPosition(clawClosed);
                    robot.setLiftPos(time + 0.25, stackOffsets[cycles] + 500, armDownBack, wristNeutral);
                })
                .waitSeconds(1)
                .setReversed(false)
                .setVelConstraint(SampleMecanumDrive.getVelocityConstraint(40, DriveConstants.MAX_ANG_VEL, DriveConstants.TRACK_WIDTH))
                .setAccelConstraint(SampleMecanumDrive.getAccelerationConstraint(40))
                .splineTo(dropPose2.vec(), dropPose2.getHeading())
                .waitSeconds(0.25)
                .addTemporalMarker(1, -2,() -> {
                    robot.setLiftPos(time, liftHigh, armDropFront, wristDropFront);
                })
                .addTemporalMarker(0, 0, () -> {
                    robot.setLiftPos(time, liftGrab + stackOffsets[cycles], armDownBack, wristNeutral);
                })
                .addTemporalMarker(1, -0.25, () -> {
                    robot.claw.setPosition(clawOpen);
                })
                .addTemporalMarker(1, 0, () -> {
                    cycles++;
                    if (cycles < totalCycles) {
                        robot.drive.followTrajectorySequenceAsync(traj3);
                    } else {
                        robot.drive.followTrajectorySequenceAsync(traj4[runCase]);
                    }
                })
                .build();
        traj4 = new TrajectorySequence[] {
                robot.drive.trajectorySequenceBuilder(dropPose2)
                        .setReversed(true)
                        .splineToSplineHeading(parkPose[0], parkPose[0].getHeading())
                        .addTemporalMarker(0, 0, () -> {
                            robot.setLiftPos(time, 0, armWait, wristNeutral);
                            readyToEnd = true;
                        })
                        .build(),
                robot.drive.trajectorySequenceBuilder(dropPose2)
                        .lineToLinearHeading(parkPose[1])
                        .addTemporalMarker(0, 0, () -> {
                            robot.setLiftPos(time, 0, armWait, wristNeutral);
                            readyToEnd = true;
                        })
                        .build(),
                robot.drive.trajectorySequenceBuilder(dropPose2)
                        .setReversed(true)
                        .splineTo(parkPose[2].vec(), parkPose[2].getHeading() + PI)
                        .addTemporalMarker(0, 0, () -> {
                            robot.setLiftPos(time, 0, armWait, wristNeutral);
                            readyToEnd = true;
                        })
                        .build()};
    }
    @Override
    public void run() {
        robot.drive.followTrajectorySequenceAsync(traj1);
        while(opModeIsActive() && !isStopRequested() && (!readyToEnd || time < robot.restTime())) {
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
