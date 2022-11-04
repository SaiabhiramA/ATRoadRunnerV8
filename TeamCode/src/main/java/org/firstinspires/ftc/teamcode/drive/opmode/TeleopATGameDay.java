package org.firstinspires.ftc.teamcode.drive.opmode;

import com.acmerobotics.roadrunner.geometry.Pose2d;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotor;

import org.firstinspires.ftc.teamcode.drive.ATGlobalStorage;
import org.firstinspires.ftc.teamcode.drive.ATRobotEnumeration;
import org.firstinspires.ftc.teamcode.drive.MecanumDriveATCancelable;
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
@TeleOp(name = "ATTeleOpMode-GameDay")
public class TeleopATGameDay extends LinearOpMode {
    TopHatAutoController tophatController;
    MecanumDriveATCancelable drive;
    ATRobotEnumeration robotMode= ATRobotEnumeration.TELE_OP_AUTO;
    Pose2d poseEstimate;
    TrajectorySequence trajSeqPark1;
    TrajectorySequence trajSeqPark2;
    TrajectorySequence trajSeqPark3;
    boolean teleOpConePickupPositioned=false;

    @Override
    public void runOpMode() throws InterruptedException {
        drive = new MecanumDriveATCancelable(hardwareMap);
        tophatController = new TopHatAutoController();
        tophatController.basicInitializeRobot(hardwareMap,telemetry,gamepad1,gamepad2,robotMode);
        drive.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        telemetry.update();
        drive.setPoseEstimate(ATGlobalStorage.currentPose);
        waitForStart();
        while (!isStopRequested()) {
            drive.update();
            poseEstimate = drive.getPoseEstimate();
            telemetry.addData("x", poseEstimate.getX());
            telemetry.addData("y", poseEstimate.getY());
            telemetry.addData("heading", Math.toDegrees(poseEstimate.getHeading()));
            telemetry.addData("robot mode", robotMode);
            telemetry.addData("AutoMode Name", ATGlobalStorage.autonModeName);
            telemetry.addData("Parking Zone", ATGlobalStorage.parkingPos);
            telemetry.update();
            this.runPlatform();
            tophatController.runTopHat();
        }

    }
    public void runPlatform(){
        if (gamepad1.right_bumper && gamepad1.left_bumper && gamepad1.x && gamepad1.left_trigger>0 && gamepad1.right_trigger>0) {
            robotMode= ATRobotEnumeration.RESET;
            tophatController.ResetTopHat();
            telemetry.addData("Reset Top Hat", "Pressed");
        }

        if (gamepad1.left_stick_y!= 0 || gamepad1.left_stick_x != 0 || gamepad1.right_stick_x !=0){
            robotMode= ATRobotEnumeration.MANUAL;
        }
        if (robotMode== ATRobotEnumeration.MANUAL) {
            drive.setWeightedDrivePower(
                    new Pose2d(
                            -gamepad1.left_stick_y,
                            -gamepad1.left_stick_x,
                            -gamepad1.right_stick_x
                    )
            );
        }
        if (gamepad1.left_bumper && gamepad1.right_bumper && gamepad1.x){
            robotMode= ATRobotEnumeration.AUTO;
            teleOpConePickupPositioned=true;
            if (ATGlobalStorage.parkingPos==ATRobotEnumeration.PARK1){
                trajSeqPark1=drive.trajectorySequenceBuilder(drive.getPoseEstimate())
                        .lineToLinearHeading(new Pose2d(13, -6, Math.toRadians(90)))
                        .lineToSplineHeading(new Pose2d(12, -32, Math.toRadians(180)))
                        .build();
                drive.followTrajectorySequenceAsync(trajSeqPark1);
            }
            else if (ATGlobalStorage.parkingPos==ATRobotEnumeration.PARK2){
                trajSeqPark2=drive.trajectorySequenceBuilder(drive.getPoseEstimate())
                        .lineToLinearHeading(new Pose2d(36, -6, Math.toRadians(90)))
                        .lineToLinearHeading(new Pose2d(36, -28, Math.toRadians(180)))
                        .lineToSplineHeading(new Pose2d(12, -32, Math.toRadians(180)))
                        .build();
                drive.followTrajectorySequenceAsync(trajSeqPark2);
            }
            else if (ATGlobalStorage.parkingPos==ATRobotEnumeration.PARK3){
                trajSeqPark3=drive.trajectorySequenceBuilder(drive.getPoseEstimate())
                        .lineToLinearHeading(new Pose2d(56, -6, Math.toRadians(90)))
                        .lineToSplineHeading(new Pose2d(60, -28, Math.toRadians(180)))
                        .lineToSplineHeading(new Pose2d(12, -32, Math.toRadians(180)))
                        .build();
                drive.followTrajectorySequenceAsync(trajSeqPark3);
            }
        }
        if (!gamepad1.left_bumper && !gamepad1.right_bumper && gamepad1.x){
            robotMode= ATRobotEnumeration.MANUAL;
            drive.breakFollowing();
            tophatController.stopTopHatMovement();
        }
    }

}
