package org.firstinspires.ftc.teamcode.drive.opmode;

import static freightfrenzy.teamcode.drive.AtomicToadsDriveConstants.MAX_ACCEL;
import static freightfrenzy.teamcode.drive.AtomicToadsDriveConstants.MAX_ANG_VEL;
import static freightfrenzy.teamcode.drive.AtomicToadsDriveConstants.TRACK_WIDTH;

import com.acmerobotics.dashboard.config.Config;
import com.acmerobotics.roadrunner.geometry.Pose2d;
import com.acmerobotics.roadrunner.geometry.Vector2d;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.Disabled;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;

import org.firstinspires.ftc.teamcode.drive.MecanumDriveAT;
import org.firstinspires.ftc.teamcode.trajectorysequence.TrajectorySequence;

/*
 * Op mode for preliminary tuning of the follower PID coefficients (located in the drive base
 * classes). The robot drives in a DISTANCE-by-DISTANCE square indefinitely. Utilization of the
 * dashboard is recommended for this tuning routine. To access the dashboard, connect your computer
 * to the RC's WiFi network. In your browser, navigate to https://192.168.49.1:8080/dash if you're
 * using the RC phone or https://192.168.43.1:8080/dash if you are using the Control Hub. Once
 * you've successfully connected, start the program, and your robot will begin driving in a square.
 * You should observe the target position (green) and your pose estimate (blue) and adjust your
 * follower PID coefficients such that you follow the target position as accurately as possible.
 * If you are using SampleMecanumDrive, you should be tuning TRANSLATIONAL_PID and HEADING_PID.
 * If you are using SampleTankDrive, you should be tuning AXIAL_PID, CROSS_TRACK_PID, and HEADING_PID.
 * These coefficients can be tuned live in dashboard.
 */
@Config
//@Autonomous(group = "drive")
@Autonomous(name = "Red Allinace Right Medium Drop")
//@Disabled
public class RedAllianceRightMedDrop extends LinearOpMode {
    @Override
    public void runOpMode() throws InterruptedException {
        MecanumDriveAT drive = new MecanumDriveAT(hardwareMap);

        Pose2d startPose = new Pose2d(30, -55, Math.toRadians(90));

        drive.setPoseEstimate(startPose);

        waitForStart();

        if (isStopRequested()) return;

       TrajectorySequence trajSeq = drive.trajectorySequenceBuilder(startPose)
                    /*.forward(DISTANCE)
                    .turn(Math.toRadians(90))
                    .forward(DISTANCE)
                    .turn(Math.toRadians(90))
                    .forward(DISTANCE)
                    .turn(Math.toRadians(90))
                    .forward(DISTANCE)
                    .turn(Math.toRadians(90))
                    .lineToConstantHeading(new Vector2d(35, -55))
                    .lineToConstantHeading(new Vector2d(35, -1))
                    .back(5)
                    .turn(Math.toRadians(180))*/
                    .splineToLinearHeading(new Pose2d(35, -40, Math.toRadians(90)),Math.toRadians(90))
                    .splineToLinearHeading(new Pose2d(35, 0, Math.toRadians(90)),Math.toRadians(90))
                    .splineToSplineHeading(new Pose2d(35, -7, Math.toRadians(270)),Math.toRadians(0))
                            /*
                            ,drive.getVelocityConstraint(30, 30, TRACK_WIDTH)
                            ,drive.getAccelerationConstraint(30)
                            )*/

                    .build();

        drive.followTrajectorySequence(trajSeq);

    }
}
