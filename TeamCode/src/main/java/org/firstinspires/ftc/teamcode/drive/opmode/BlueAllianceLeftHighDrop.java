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

@Config
//@Autonomous(group = "drive")
@Autonomous(name = "BlueAllinaceLeftHighDropDS")
//@Disabled
public class BlueAllianceLeftHighDrop extends LinearOpMode {
    MecanumDriveAT drive;
    TopHatAutoController tophatController;
    ATTensorFlowDefaultDetection ATObjectDetection;
    ATRobotEnumeration parkingZone=ATRobotEnumeration.SUBSTATION;
    Pose2d poseEstimate;
    boolean isAutonConePickupReady=false;
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
        TrajectorySequence trajSeqConePickup = drive.trajectorySequenceBuilder(startPose)
                .addTemporalMarker(.001, ()->{
                    tophatController.setRobotMode(ATRobotEnumeration.SET_BLUE_LEFT_PRELOADED_CONE);
                    tophatController.blueAllianceLeftAutonHigh();})
                .waitSeconds(2.5)
                .splineToConstantHeading(new Vector2d(36, 58), Math.toRadians(-90),drive.getVelocityConstraint(25, MAX_ANG_VEL, TRACK_WIDTH), drive.getAccelerationConstraint(MAX_ACCEL))
                .lineTo(new Vector2d(36, 4), drive.getVelocityConstraint(25, MAX_ANG_VEL, TRACK_WIDTH), drive.getAccelerationConstraint(MAX_ACCEL))
                .waitSeconds(1)
                .lineTo(new Vector2d(33, 4), drive.getVelocityConstraint(25, MAX_ANG_VEL, TRACK_WIDTH), drive.getAccelerationConstraint(MAX_ACCEL))
                .addDisplacementMarker(()->{
                    tophatController.setRobotMode(ATRobotEnumeration.DROP_BLUE_LEFT_PRELOADED_CONE);
                    tophatController.blueAllianceLeftAutonHigh();})
                .waitSeconds(1)
                .lineTo(new Vector2d(45, 8), drive.getVelocityConstraint(25, MAX_ANG_VEL, TRACK_WIDTH), drive.getAccelerationConstraint(MAX_ACCEL))
                .build();
        while (opModeInInit()) {
            parkingZone = ATObjectDetection.detectObjectLabel();
            telemetry.addData("Parking Zone", parkingZone);
            telemetry.addData("Get Runtime", this.getRuntime());
            if (this.getRuntime()>30){
                telemetry.addData("ATOMIC TOADS, You may Now PRESS PLAY - WISH YOU GOOD LUCK", this.getRuntime());
            }
            telemetry.update();
        }
        waitForStart();
        if (isStopRequested()) return;
        drive.followTrajectorySequence(trajSeqConePickup);

        if (isAutonConePickupReady) {
            tophatController.setRobotMode(ATRobotEnumeration.AUTO_BLUE_LEFT_HIGH_PICK_CONE);
            while ((!isStopRequested()) && !tophatController.areFiveConesDone()) {
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
        }
        else{
            tophatController.setRobotMode(ATRobotEnumeration.DROP_BLUE_LEFT_PRELOADED_CONE);
            tophatController.setTopHatPosition(.1,false,4300,-1800,945);
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
                        .lineToConstantHeading(new Vector2d(60, 10),drive.getVelocityConstraint(45, MAX_ANG_VEL, TRACK_WIDTH), drive.getAccelerationConstraint(MAX_ACCEL))
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
                        .lineToConstantHeading(new Vector2d(12, 10),drive.getVelocityConstraint(45, MAX_ANG_VEL, TRACK_WIDTH), drive.getAccelerationConstraint(MAX_ACCEL))
                        .build();
                drive.followTrajectorySequence(trajSeqParking);
            }
            else if (parkingZone==ATRobotEnumeration.SUBSTATION){
                trajSeqParking=drive.trajectorySequenceBuilder(drive.getPoseEstimate())
                        .lineToConstantHeading(new Vector2d(12, 10),drive.getVelocityConstraint(45, MAX_ANG_VEL, TRACK_WIDTH), drive.getAccelerationConstraint(MAX_ACCEL))
                        .lineToSplineHeading(new Pose2d(10, 60, Math.toRadians(90)))
                        .build();
                drive.followTrajectorySequence(trajSeqParking);
            }
        setRobotStateInStorage();
    }
    private void setRobotStateInStorage(){
        ATGlobalStorage.autonModeName=ATRobotEnumeration.BLUE_LEFT_HIGH_DROP;
        ATGlobalStorage.currentPose=drive.getPoseEstimate();
        ATGlobalStorage.parkingPos=parkingZone;
        ATGlobalStorage.allianceName=ATRobotEnumeration.BLUE_ALLIANCE;
    }
}
