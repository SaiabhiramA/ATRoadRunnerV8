package org.firstinspires.ftc.teamcode.drive.opmode;

import static freightfrenzy.teamcode.drive.AtomicToadsDriveConstants.MAX_ACCEL;
import static freightfrenzy.teamcode.drive.AtomicToadsDriveConstants.MAX_ANG_VEL;
import static freightfrenzy.teamcode.drive.AtomicToadsDriveConstants.TRACK_WIDTH;

import com.acmerobotics.dashboard.config.Config;
import com.acmerobotics.roadrunner.geometry.Pose2d;
import com.acmerobotics.roadrunner.geometry.Vector2d;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;

import org.firstinspires.ftc.teamcode.drive.ATGlobalStorage;
import org.firstinspires.ftc.teamcode.drive.ATRobotEnumeration;
import org.firstinspires.ftc.teamcode.drive.ATTensorFlowDefaultDetection;
import org.firstinspires.ftc.teamcode.drive.MecanumDriveAT;
import org.firstinspires.ftc.teamcode.drive.TopHatAutoController;
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
@Autonomous(name = "BlueAllinaceLeftHighDrop")
//@Disabled
public class BlueAllianceLeftHighDrop extends LinearOpMode {
    MecanumDriveAT drive;
    TopHatAutoController tophatController;
    ATTensorFlowDefaultDetection ATObjectDetection;
    ATRobotEnumeration parkingZone;
    double initTimeElapsed;
    Pose2d poseEstimate;
    @Override
    public void runOpMode() throws InterruptedException {
        tophatController=new TopHatAutoController();
        ATObjectDetection = new ATTensorFlowDefaultDetection();
        ATObjectDetection.initalizeTensorFlow(hardwareMap, telemetry, ATRobotEnumeration.AUTO_RED_RIGHT_HIGH_SETUP);
        parkingZone = ATObjectDetection.detectObjectLabel();
        tophatController.fullyInitializeRobot(telemetry, gamepad1, gamepad2, ATRobotEnumeration.RESET, hardwareMap);
        drive = new MecanumDriveAT(hardwareMap);
        Pose2d startPose = new Pose2d(40, 60, Math.toRadians(270));
        drive.setPoseEstimate(startPose);
        parkingZone=ATRobotEnumeration.SUBSTATION;
        setRobotStateInStorage();
        while (this.getRuntime()<20){
            parkingZone = ATObjectDetection.detectObjectLabel();
            telemetry.addData("Parking Zone", parkingZone);
            telemetry.addData("Get Runtime", this.getRuntime());
            telemetry.update();
            sleep(500);
        }
        initTimeElapsed = this.getRuntime();
        waitForStart();
        if (isStopRequested()) return;
        TrajectorySequence trajSeqConePickup = drive.trajectorySequenceBuilder(startPose)
                .splineToConstantHeading(new Vector2d(36, 57), Math.toRadians(-90), drive.getVelocityConstraint(45, MAX_ANG_VEL, TRACK_WIDTH), drive.getAccelerationConstraint(MAX_ACCEL))
                .splineToConstantHeading(new Vector2d(36, 30), Math.toRadians(-90), drive.getVelocityConstraint(45, MAX_ANG_VEL, TRACK_WIDTH), drive.getAccelerationConstraint(MAX_ACCEL))
                .splineToConstantHeading(new Vector2d(44, 10), Math.toRadians(0), drive.getVelocityConstraint(45, MAX_ANG_VEL, TRACK_WIDTH), drive.getAccelerationConstraint(MAX_ACCEL))
                .addTemporalMarker(.1, ()->{
                    tophatController.setRobotMode(ATRobotEnumeration.AUTO_BLUE_LEFT_HIGH_SETUP);
                    tophatController.blueAllianceLeftAutonHigh();})
                .build();
        drive.followTrajectorySequence(trajSeqConePickup);
        tophatController.setRobotMode(ATRobotEnumeration.AUTO_BLUE_LEFT_HIGH_PICK_CONE);
        poseEstimate = drive.getPoseEstimate();

        /*while ((!isStopRequested())&& !tophatController.areFiveConesDone())
        {
            tophatController.blueAllianceLeftAutonHigh();
            poseEstimate = drive.getPoseEstimate();
            telemetry.addData("x", poseEstimate.getX());
            telemetry.addData("y", poseEstimate.getY());
            telemetry.addData("heading", Math.toDegrees(poseEstimate.getHeading()));
            telemetry.addData("robot mode", tophatController.getRobotMode());
            telemetry.addData("Parking Zone", parkingZone);
            telemetry.addData("Get Runtime", this.getRuntime());
            telemetry.addData("In While Loop", "YES");
            telemetry.addData("Stop  Requested", isStopRequested());
            telemetry.addData("are FiveCones Done", tophatController.areFiveConesDone());
            telemetry.update();
        }
        TrajectorySequence trajSeqParking;

            if (parkingZone==ATRobotEnumeration.PARK1){
                trajSeqParking=drive.trajectorySequenceBuilder(drive.getPoseEstimate())
                        .lineToConstantHeading(new Vector2d(13, 10),drive.getVelocityConstraint(45, MAX_ANG_VEL, TRACK_WIDTH), drive.getAccelerationConstraint(MAX_ACCEL))
                        .build();
                drive.followTrajectorySequence(trajSeqParking);
            }
            else if (parkingZone==ATRobotEnumeration.PARK2){
                trajSeqParking=drive.trajectorySequenceBuilder(drive.getPoseEstimate())
                        .lineToConstantHeading(new Vector2d(36, 10),drive.getVelocityConstraint(45, MAX_ANG_VEL, TRACK_WIDTH), drive.getAccelerationConstraint(MAX_ACCEL))
                        .build();
                drive.followTrajectorySequence(trajSeqParking);
            }
            else if (parkingZone==ATRobotEnumeration.PARK3){
                trajSeqParking=drive.trajectorySequenceBuilder(drive.getPoseEstimate())
                        .lineToConstantHeading(new Vector2d(60, 10),drive.getVelocityConstraint(45, MAX_ANG_VEL, TRACK_WIDTH), drive.getAccelerationConstraint(MAX_ACCEL))
                        .build();
                drive.followTrajectorySequence(trajSeqParking);
            }
            else if (parkingZone==ATRobotEnumeration.SUBSTATION){
                trajSeqParking=drive.trajectorySequenceBuilder(drive.getPoseEstimate())
                        .lineToConstantHeading(new Vector2d(13, 10),drive.getVelocityConstraint(45, MAX_ANG_VEL, TRACK_WIDTH), drive.getAccelerationConstraint(MAX_ACCEL))
                        .lineToSplineHeading(new Pose2d(10, 60, Math.toRadians(90)))
                        .build();
                drive.followTrajectorySequence(trajSeqParking);
            }*/
        setRobotStateInStorage();
        while (!isStopRequested()){
            poseEstimate = drive.getPoseEstimate();
            telemetry.addData("Outside While Loop", "YES");
            telemetry.addData("x", poseEstimate.getX());
            telemetry.addData("y", poseEstimate.getY());
            telemetry.addData("heading", Math.toDegrees(poseEstimate.getHeading()));
            telemetry.addData("*** Moving to Parking Zone Now ***", parkingZone);
            telemetry.addData("robot mode", tophatController.getRobotMode());
            telemetry.addData("Parking Zone", parkingZone);
            telemetry.update();
        }
    }
    private void setRobotStateInStorage(){
        ATGlobalStorage.autonModeName=ATRobotEnumeration.RED_RIGHT_HIGH_DROP;
        ATGlobalStorage.currentPose=drive.getPoseEstimate();
        ATGlobalStorage.parkingPos=parkingZone;
    }

}
