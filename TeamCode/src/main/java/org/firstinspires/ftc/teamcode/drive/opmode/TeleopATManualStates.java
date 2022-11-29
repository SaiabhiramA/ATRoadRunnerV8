package org.firstinspires.ftc.teamcode.drive.opmode;

import com.acmerobotics.roadrunner.geometry.Pose2d;
import com.qualcomm.robotcore.eventloop.opmode.Disabled;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotor;

import org.firstinspires.ftc.teamcode.drive.ATRobotEnumeration;
import org.firstinspires.ftc.teamcode.drive.MecanumDriveAT;
import org.firstinspires.ftc.teamcode.drive.TopHatAutoController;
import org.firstinspires.ftc.teamcode.drive.TopHatAutoControllerStates;

/**
 * This is a simple teleop routine for testing localization. Drive the robot around like a normal
 * teleop routine and make sure the robot's estimated pose matches the robot's actual pose (slight
 * errors are not out of the ordinary, especially with sudden drive motions). The goal of this
 * exercise is to ascertain whether the localizer has been configured properly (note: the pure
 * encoder localizer heading may be significantly off if the track width has not been tuned).
 */
//@TeleOp(group = "drive")
@TeleOp(name = "ATTeleOpMode-FullyManualStates")
public class TeleopATManualStates extends LinearOpMode {
    TopHatAutoControllerStates tophatController;
    MecanumDriveAT drive;
    ATRobotEnumeration robotMode= ATRobotEnumeration.MANUAL;
    Pose2d poseEstimate;

    @Override
    public void runOpMode() throws InterruptedException {
        drive = new MecanumDriveAT(hardwareMap);
        tophatController = new TopHatAutoControllerStates();
        tophatController.fullyInitializeRobot(telemetry, gamepad1, gamepad2, robotMode, hardwareMap);
        drive.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        telemetry.update();
        waitForStart();
        while (opModeIsActive() && !isStopRequested())  {
            this.runPlatform();
            tophatController.runTopHat();
            poseEstimate = drive.getPoseEstimate();
            telemetry.addData("x", poseEstimate.getX());
            telemetry.addData("y", poseEstimate.getY());
            telemetry.addData("heading", Math.toDegrees(poseEstimate.getHeading()));
            telemetry.addData("robot mode", robotMode);
            telemetry.update();
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
                            -0.5*gamepad1.left_stick_y,
                            -0.5*gamepad1.left_stick_x,
                            -0.5*gamepad1.right_stick_x
                    )
            );
            drive.update();
        }
       }

}
