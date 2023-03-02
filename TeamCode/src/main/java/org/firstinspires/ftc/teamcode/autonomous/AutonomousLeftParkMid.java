package org.firstinspires.ftc.teamcode.autonomous;
import static org.firstinspires.ftc.teamcode.classes.ValueStorage.*;
import static java.lang.Math.*;
import com.acmerobotics.roadrunner.geometry.Pose2d;
import com.acmerobotics.roadrunner.geometry.Vector2d;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import org.firstinspires.ftc.teamcode.drive.DriveConstants;
import org.firstinspires.ftc.teamcode.drive.SampleMecanumDrive;
import org.firstinspires.ftc.teamcode.trajectorysequence.TrajectorySequence;
@Autonomous(name = "LeftParkMid")
public class AutonomousLeftParkMid extends AbstractAutonomous {
    Pose2d dropPose = new Pose2d(27, 29, PI + 1);
    Pose2d[] parkPose = new Pose2d[] {new Pose2d(60, 36, -PI / 2), new Pose2d(36, 36, -PI / 2), new Pose2d(12, 36, -PI / 2)};
    TrajectorySequence traj1;
    TrajectorySequence[] traj2;
    boolean readyToEnd = false;
    boolean parkDone = false;
    @Override
    public void initialize() {
        traj1 = robot.drive.trajectorySequenceBuilder(initPose())
                .setVelConstraint(SampleMecanumDrive.getVelocityConstraint(30, DriveConstants.MAX_ANG_VEL, DriveConstants.TRACK_WIDTH))
                .setAccelConstraint(SampleMecanumDrive.getAccelerationConstraint(30))
                .splineTo(new Vector2d(35, 50), -PI / 2)
                .splineTo(dropPose.vec(), dropPose.getHeading())
                .addTemporalMarker(1, -1.5, () -> {
                    robot.setLiftPos(time, liftMid, armDropFront, wristDropFront);
                })
                .addTemporalMarker(1, 0, () -> {
                    robot.claw.setPosition(clawOpen);
                    robot.drive.followTrajectorySequenceAsync(traj2[runCase - 1]);
                })
                .build();
        traj2 = new TrajectorySequence[] {
                robot.drive.trajectorySequenceBuilder(dropPose)
                        .setVelConstraint(SampleMecanumDrive.getVelocityConstraint(30, DriveConstants.MAX_ANG_VEL, DriveConstants.TRACK_WIDTH))
                        .setAccelConstraint(SampleMecanumDrive.getAccelerationConstraint(30))
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
                        .setVelConstraint(SampleMecanumDrive.getVelocityConstraint(30, DriveConstants.MAX_ANG_VEL, DriveConstants.TRACK_WIDTH))
                        .setAccelConstraint(SampleMecanumDrive.getAccelerationConstraint(30))
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
                        .setVelConstraint(SampleMecanumDrive.getVelocityConstraint(30, DriveConstants.MAX_ANG_VEL, DriveConstants.TRACK_WIDTH))
                        .setAccelConstraint(SampleMecanumDrive.getAccelerationConstraint(30))
                        .lineToSplineHeading(parkPose[1])
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
