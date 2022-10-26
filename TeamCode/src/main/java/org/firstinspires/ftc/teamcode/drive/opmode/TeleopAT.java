package org.firstinspires.ftc.teamcode.drive.opmode;

import com.acmerobotics.roadrunner.geometry.Pose2d;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotor;

import org.firstinspires.ftc.teamcode.drive.MecanumDriveAT;
import org.firstinspires.ftc.teamcode.drive.TopHatAutoController;
import org.firstinspires.ftc.teamcode.trajectorysequence.TrajectorySequence;

/**
 * This is a simple teleop routine for testing localization. Drive the robot around like a normal
 * teleop routine and make sure the robot's estimated pose matches the robot's actual pose (slight
 * errors are not out of the ordinary, especially with sudden drive motions). The goal of this
 * exercise is to ascertain whether the localizer has been configured properly (note: the pure
 * encoder localizer heading may be significantly off if the track width has not been tuned).
 */
//@TeleOp(group = "drive")
@TeleOp(name = "TeleOpAT")
public class TeleopAT extends LinearOpMode {
    TopHatAutoController tophatController;
    MecanumDriveAT drive;
    @Override
    public void runOpMode() throws InterruptedException {
        drive = new MecanumDriveAT(hardwareMap);
        //tophatController = new TopHatAutoController();
        //tophatController.initializeRobot(hardwareMap,drive,telemetry,gamepad1,gamepad2,"");
        drive.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);

        waitForStart();

        while (!isStopRequested()) {
            /*drive.setWeightedDrivePower(
                    new Pose2d(
                            -gamepad1.left_stick_y,
                            -gamepad1.left_stick_x,
                            -gamepad1.right_stick_x
                    )
            );*/
            //tophatController.runTopHat();
            this.runPlatform();

            //drive.update();
            Pose2d poseEstimate = drive.getPoseEstimate();
            telemetry.addData("x", poseEstimate.getX());
            telemetry.addData("y", poseEstimate.getY());
            telemetry.addData("heading", Math.toDegrees(poseEstimate.getHeading()));
            telemetry.update();
            //sleep(20); // check if we need this is all action are not being performed.
        }

    }
    public void runPlatform(){
        if (gamepad1.right_bumper && gamepad1.left_bumper && gamepad1.x && gamepad1.left_trigger>0 && gamepad1.right_trigger>0) {
            tophatController.ResetTopHat();
            telemetry.addData("Reset Top Hat", "Pressed");
        }
        if (gamepad1.y && gamepad1.right_bumper && gamepad1.left_bumper){

                    // Set bot constraints: maxVel, maxAccel, maxAngVel, maxAngAccel, track width
                    //.setConstraints(60, 60, Math.toRadians(180), Math.toRadians(0), 15)
                    Pose2d startPose = new Pose2d(30, -55, Math.toRadians(0));
                    drive.setPoseEstimate(startPose);
                    TrajectorySequence trajSeq = drive.trajectorySequenceBuilder(startPose)
                    //drive.followTrajectorySequence(drive ->
                            //drive.trajectorySequenceBuilder(new Pose2d(-35, 60, Math.toRadians(270)))
                                    //.forward(30)
                                    //.splineTo(50,50)
                                    .lineToLinearHeading(new Pose2d(35, -55, Math.toRadians(0)))
                                    .lineToLinearHeading(new Pose2d(35, -4.5, Math.toRadians(0)))
                                    .turn(Math.toRadians(180))
                                    //.forward(30)
                                    //.turn(Math.toRadians(90))
                                    //.forward(30)
                                    //.turn(Math.toRadians(90))
                                    //.forward(30)
                                    //.turn(Math.toRadians(90))
                                    .build();
                    drive.followTrajectorySequence(trajSeq);
        }

    }

}
