package org.firstinspires.ftc.teamcode.drive.opmode;

import com.acmerobotics.roadrunner.geometry.Pose2d;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotor;

import org.firstinspires.ftc.teamcode.drive.ATRobotMode;
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
@TeleOp(name = "AT Tele Op Mode")
public class TeleopAT extends LinearOpMode {
    TopHatAutoController tophatController;
    MecanumDriveAT drive;
    ATRobotMode robotMode=ATRobotMode.MANUAL;
    @Override
    public void runOpMode() throws InterruptedException {
        drive = new MecanumDriveAT(hardwareMap);
        tophatController = new TopHatAutoController();

        tophatController.initializeRobot(hardwareMap,drive,telemetry,gamepad1,gamepad2,"",robotMode);
        drive.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        telemetry.update();

        waitForStart();

        while (!isStopRequested()) {

            this.runPlatform();
            tophatController.runTopHat();
            Pose2d poseEstimate = drive.getPoseEstimate();
            telemetry.addData("x", poseEstimate.getX());
            telemetry.addData("y", poseEstimate.getY());
            telemetry.addData("heading", Math.toDegrees(poseEstimate.getHeading()));
            telemetry.addData("robot mode", robotMode);
            telemetry.update();
            //sleep(20); // check if we need this is all action are not being performed.
        }

    }
    public void runPlatform(){
        if (gamepad1.right_bumper && gamepad1.left_bumper && gamepad1.x && gamepad1.left_trigger>0 && gamepad1.right_trigger>0) {
            robotMode=ATRobotMode.RESET;
            tophatController.ResetTopHat();
            telemetry.addData("Reset Top Hat", "Pressed");
        }

        if (gamepad1.left_stick_y!= 0 || gamepad1.left_stick_x != 0 || gamepad1.right_stick_x !=0){
            robotMode=ATRobotMode.MANUAL;
        }
        if (robotMode==ATRobotMode.MANUAL) {
            drive.setWeightedDrivePower(
                    new Pose2d(
                            -gamepad1.left_stick_y,
                            -gamepad1.left_stick_x,
                            -gamepad1.right_stick_x
                    )
            );
            drive.update();
        }
        if (gamepad1.b && gamepad1.right_bumper && gamepad1.left_bumper){
                    robotMode=ATRobotMode.AUTO_RED_RIGHT_MEDIUM_SETUP;
                    Pose2d startPose = new Pose2d(30, -55, Math.toRadians(90));
                    drive.setPoseEstimate(startPose);

                    TrajectorySequence trajSeq = drive.trajectorySequenceBuilder(startPose)
                                    .splineToLinearHeading(new Pose2d(33, -40, Math.toRadians(90)),Math.toRadians(90))
                                    .addTemporalMarker(.1, ()->{
                                        tophatController.setRobotMode(robotMode);
                                        tophatController.redAllianceRightAutonMedium();})
                                    //.waitSeconds(3)robotMode
                                    .splineToLinearHeading(new Pose2d(33, -2, Math.toRadians(90)),Math.toRadians(90))
                                    .splineToSplineHeading(new Pose2d(42, -12, Math.toRadians(270)),Math.toRadians(0))
                                    //.splineToLinearHeading(new Pose2d(42, -7, Math.toRadians(270)),Math.toRadians(90))
                                    //changed from 42
                                    .back(5)
                                    .build();
                    drive.followTrajectorySequence(trajSeq);
                    robotMode=ATRobotMode.AUTO_RED_RIGHT_MEDIUM_PICK_CONE;
                    tophatController.setRobotMode(robotMode);
                    //tophatController.redAllianceRightAutonMedium(robotMode);
        }
        if (gamepad1.y && gamepad1.right_bumper && gamepad1.left_bumper) {
            robotMode = ATRobotMode.AUTO_RED_RIGHT_HIGH_PICK_CONE;
            tophatController.setRobotMode(robotMode);
            //tophatController.redAllianceRightAutonHigh();
        }

    }

}
