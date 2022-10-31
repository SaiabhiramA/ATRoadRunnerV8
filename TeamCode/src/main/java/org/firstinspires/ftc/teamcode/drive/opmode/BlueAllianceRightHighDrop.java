package org.firstinspires.ftc.teamcode.drive.opmode;

import com.acmerobotics.dashboard.config.Config;
import com.acmerobotics.roadrunner.geometry.Pose2d;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;

import org.firstinspires.ftc.teamcode.drive.ATRobotMode;
import org.firstinspires.ftc.teamcode.drive.ATTensorFlowDefaultDetection;
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
@Autonomous(name = "Blue Allinace Right High Drop")
//@Disabled
public class BlueAllianceRightHighDrop extends LinearOpMode {
    MecanumDriveAT drive;
    //TopHatAutoController tophatController;
    ATTensorFlowDefaultDetection ATObjectDetection;
    ATRobotMode parkingZone;
    double initTimeElapsed;


    @Override
    public void runOpMode() throws InterruptedException {

        //tophatController=new TopHatAutoController();
        //tophatController.initializeRobot(hardwareMap,drive,telemetry,gamepad1,gamepad2,"", ATRobotMode.RESET);
        ATObjectDetection = new ATTensorFlowDefaultDetection();
        ATObjectDetection.initalizeTensorFlow(hardwareMap, telemetry, ATRobotMode.AUTO_RED_RIGHT_HIGH_SETUP);
        drive = new MecanumDriveAT(hardwareMap);
        Pose2d startPose = new Pose2d(30, 55, Math.toRadians(270));
        drive.setPoseEstimate(startPose);
       while (!isStarted()){
            parkingZone = ATObjectDetection.detectObjectLabel();
            telemetry.addData("Parking Zone", parkingZone);
            telemetry.addData("Get Runtime", this.getRuntime());
            telemetry.update();
            sleep(500);

        }


        initTimeElapsed = this.getRuntime();
        //waitForStart();
        if (isStopRequested()) return;

        TrajectorySequence trajSeqConePickup = drive.trajectorySequenceBuilder(startPose)
                .splineToLinearHeading(new Pose2d(33, 40, Math.toRadians(270)), Math.toRadians(270))
                //.addTemporalMarker(.1, ()->{
                //tophatController.setRobotMode(ATRobotMode.AUTO_RED_RIGHT_HIGH_SETUP);
                //tophatController.redAllianceRightAutonHigh();})
                .lineToLinearHeading(new Pose2d(33, 2, Math.toRadians(270)))
                .lineToLinearHeading(new Pose2d(33, 5.75, Math.toRadians(270)))
                .lineToLinearHeading(new Pose2d(41, 5.75, Math.toRadians(270)))
                .build();
        drive.followTrajectorySequence(trajSeqConePickup);
        //tophatController.setRobotMode(ATRobotMode.AUTO_RED_RIGHT_HIGH_PICK_CONE);

        Pose2d poseEstimate = drive.getPoseEstimate();
        telemetry.addData("x", poseEstimate.getX());
        telemetry.addData("y", poseEstimate.getY());
        telemetry.addData("heading", Math.toDegrees(poseEstimate.getHeading()));
        telemetry.addData("Parking Zone", parkingZone);
        //telemetry.addData("robot mode", tophatController.getRobotMode());
        telemetry.update();
        TrajectorySequence trajSeqParking;
        while (getRuntime()<initTimeElapsed + 20){
            telemetry.addData("Get Runtime", initTimeElapsed+this.getRuntime());
            telemetry.update();
        }
            if (parkingZone==ATRobotMode.PARK1){
                trajSeqParking=drive.trajectorySequenceBuilder(drive.getPoseEstimate())
                        .lineToLinearHeading(new Pose2d(13, 6, Math.toRadians(270)))
                        .build();
                drive.followTrajectorySequence(trajSeqParking);
            }
            else if (parkingZone==ATRobotMode.PARK2){
                trajSeqParking=drive.trajectorySequenceBuilder(drive.getPoseEstimate())
                        .lineToLinearHeading(new Pose2d(36, 6, Math.toRadians(270)))
                        .build();
                drive.followTrajectorySequence(trajSeqParking);
            }
            else if (parkingZone==ATRobotMode.PARK3){
                trajSeqParking=drive.trajectorySequenceBuilder(drive.getPoseEstimate())
                        .lineToLinearHeading(new Pose2d(55, 6, Math.toRadians(270)))
                        .build();
                drive.followTrajectorySequence(trajSeqParking);
            }
            else if (parkingZone==ATRobotMode.SUBSTATION){
                trajSeqParking=drive.trajectorySequenceBuilder(drive.getPoseEstimate())
                        .lineToLinearHeading(new Pose2d(10, 6, Math.toRadians(90)))
                        .lineToLinearHeading(new Pose2d(10, 53, Math.toRadians(90)))
                        .build();
                drive.followTrajectorySequence(trajSeqParking);
            }

            //telemetry.addData("Get Runtime", this.getRuntime());
            //telemetry.update();

        /*while ((!isStopRequested()) || !tophatController.areFiveConesDone()) {
            tophatController.redAllianceRightAutonHigh();
            telemetry.addData("x", poseEstimate.getX());
            telemetry.addData("y", poseEstimate.getY());
            telemetry.addData("heading", Math.toDegrees(poseEstimate.getHeading()));
            telemetry.addData("robot mode", tophatController.getRobotMode());
            telemetry.update();
        }
*/
    }
}
