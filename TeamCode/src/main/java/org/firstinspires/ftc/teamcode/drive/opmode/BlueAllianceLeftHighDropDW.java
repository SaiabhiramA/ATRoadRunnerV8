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

import org.firstinspires.ftc.teamcode.drive.ATConstants;
import org.firstinspires.ftc.teamcode.drive.ATGlobalStorage;
import org.firstinspires.ftc.teamcode.drive.ATRobotEnumeration;
import org.firstinspires.ftc.teamcode.drive.MecanumDriveAT;
import org.firstinspires.ftc.teamcode.drive.MecanumDriveATWheels;
import org.firstinspires.ftc.teamcode.drive.TopHatAutoControllerStates;
import org.firstinspires.ftc.teamcode.trajectorysequence.TrajectorySequence;

@Config
//@Autonomous(group = "drive")
@Autonomous(name = "BlueLeftHighDropAT-DeadWheel")

public class BlueAllianceLeftHighDropDW extends LinearOpMode {
    MecanumDriveATWheels drive;
    TopHatAutoControllerStates tophatController;
    ATAprilTag ATObjectDetection;
    ATRobotEnumeration parkingZone=ATRobotEnumeration.SUBSTATION;
    Pose2d poseEstimate;
    boolean isAutonConePickupReady=true;
    @Override
    public void runOpMode() throws InterruptedException {
        tophatController=new TopHatAutoControllerStates();
        tophatController.robotMode=ATRobotEnumeration.FULL_AUTON;
        ATObjectDetection =  new ATAprilTag();
        ATObjectDetection.initalizeTensorFlow(hardwareMap, telemetry, ATRobotEnumeration.AUTO_BLUE_LEFT_HIGH_SETUP);
        //parkingZone = ATObjectDetection.detectObjectLabel();
        tophatController.fullyInitializeRobot(telemetry, gamepad1, gamepad2, ATRobotEnumeration.RESET, hardwareMap);
        tophatController.setTopHatSpeed(ATRobotEnumeration.TOPHAT_LOW_AUTON_SPEED);
        drive = new MecanumDriveATWheels(hardwareMap);
        Pose2d startPose = new Pose2d(40, 60, Math.toRadians(270));
        drive.setPoseEstimate(startPose);
        //parkingZone=ATRobotEnumeration.SUBSTATION;
        setRobotStateInStorage();
        TrajectorySequence trajSeqConePickup = drive.trajectorySequenceBuilder(startPose)
                .addTemporalMarker(.001, ()->{
                    tophatController.setTophatAction(ATRobotEnumeration.SET_BLUE_LEFT_PRELOADED_CONE);
                    tophatController.blueAllianceLeftAutonHigh();})
                .splineToConstantHeading(new Vector2d(36,58), Math.toRadians(-90),drive.getVelocityConstraint(15, MAX_ANG_VEL, TRACK_WIDTH), drive.getAccelerationConstraint(MAX_ACCEL))
                .addTemporalMarker(1, ()->{
                    if (tophatController.tophatAction==ATRobotEnumeration.SET_BLUE_LEFT_PRELOADED_CONE) {
                        tophatController.blueAllianceLeftAutonHigh();
                    }
                    })
                .splineToConstantHeading(new Vector2d(36,40), Math.toRadians(-90),drive.getVelocityConstraint(15, MAX_ANG_VEL, TRACK_WIDTH), drive.getAccelerationConstraint(MAX_ACCEL))
                .addTemporalMarker(2, ()->{
                    if (tophatController.tophatAction==ATRobotEnumeration.SET_BLUE_LEFT_PRELOADED_CONE) {
                        tophatController.blueAllianceLeftAutonHigh();
                    }
                })
                .splineToConstantHeading(new Vector2d(33,18.5), Math.toRadians(-90),drive.getVelocityConstraint(15, MAX_ANG_VEL, TRACK_WIDTH), drive.getAccelerationConstraint(MAX_ACCEL))
                .addTemporalMarker(3, ()->{
                    if (tophatController.tophatAction==ATRobotEnumeration.SET_BLUE_LEFT_PRELOADED_CONE) {
                        tophatController.blueAllianceLeftAutonHigh();
                    }
                })
                .addTemporalMarker(4, ()->{
                    tophatController.setTophatAction(ATRobotEnumeration.DROP_BLUE_LEFT_PRELOADED_CONE);
                    tophatController.blueAllianceLeftAutonHigh();})
                .waitSeconds(1)
                .splineToConstantHeading(new Vector2d(40,11), Math.toRadians(0),drive.getVelocityConstraint(15, MAX_ANG_VEL, TRACK_WIDTH), drive.getAccelerationConstraint(MAX_ACCEL))
                //.lineToConstantHeading(new Vector2d(40,11),drive.getVelocityConstraint(25, MAX_ANG_VEL, TRACK_WIDTH), drive.getAccelerationConstraint(MAX_ACCEL))
                .build();

        //TrajectorySequence coneAdjust = drive.trajectorySequenceBuilder(startPose)

        while (opModeInInit()) {
            parkingZone = ATObjectDetection.detectObjectLabel();
            telemetry.addData("Parking Zone", parkingZone);
            telemetry.addData("Get Runtime", this.getRuntime());
            if (this.getRuntime()>10){
                telemetry.addData("ATOMIC TOADS, You may Now PRESS PLAY - WISH YOU GOOD LUCK", this.getRuntime());
            }
            telemetry.update();
        }
        waitForStart();
        if (isStopRequested()) return;
        drive.followTrajectorySequence(trajSeqConePickup);
        tophatController.setTopHatSpeed(ATRobotEnumeration.TOPHAT_MEDIUM_SPEED);
        if (isAutonConePickupReady) {
            tophatController.setTophatAction(ATRobotEnumeration.AUTO_BLUE_LEFT_HIGH_PICK_CONE);
            while ((!isStopRequested()) && !tophatController.areFiveConesDone()) {
                tophatController.blueAllianceLeftAutonHigh();
                poseEstimate = drive.getPoseEstimate();
                telemetry.addData("x", poseEstimate.getX());
                telemetry.addData("y", poseEstimate.getY());
                telemetry.addData("heading", Math.toDegrees(poseEstimate.getHeading()));
                telemetry.addData("robot mode", tophatController.getTophatAction());
                telemetry.addData("Parking Zone", parkingZone);
                telemetry.addData("Get Runtime", this.getRuntime());
                telemetry.addData("In While Loop", "YES");
                telemetry.addData("Stop  Requested", isStopRequested());
                telemetry.addData("Existing TopHat Speed", tophatController.topHatSpeed);
                telemetry.addData("Selected TopHat Speed", tophatController.selectedTopHatSpeed);
                telemetry.addData("are FiveCones Done", tophatController.areFiveConesDone());
                telemetry.update();
            }
        }
        else {
            tophatController.moveTopHatPosition(.9, false, 4250*tophatController.armMultiplier, -1800*tophatController.elbowMultiplier, 1800*tophatController.turnTableMultiplier);
            while ((!isStopRequested()) && !tophatController.isTopHatMoveCompleted(4250*tophatController.armMultiplier,-1800*tophatController.elbowMultiplier,1800*tophatController.turnTableMultiplier)) {
                poseEstimate = drive.getPoseEstimate();
                telemetry.addData("x", poseEstimate.getX());
                telemetry.addData("y", poseEstimate.getY());
                telemetry.addData("heading", Math.toDegrees(poseEstimate.getHeading()));
                telemetry.addData("robot mode", tophatController.getTophatAction());
                telemetry.addData("In While Loop", "YES");
                telemetry.update();
            }
            tophatController.setTophatAction(ATRobotEnumeration.AUTO_BLUE_LEFT_HIGH_PARK);
        }
        if (tophatController.tophatAction==ATRobotEnumeration.AUTO_BLUE_LEFT_HIGH_PARK){
            while ((!isStopRequested()) && !tophatController.isTopHatInParkingPosition()){
                telemetry.addData("Top Hat is in Parking Mode", "YES");
            }
        }

        TrajectorySequence trajSeqParking;
            if (parkingZone==ATRobotEnumeration.PARK1){
                trajSeqParking=drive.trajectorySequenceBuilder(drive.getPoseEstimate())
                        .lineToConstantHeading(new Vector2d(57, 11))//,drive.getVelocityConstraint(45, MAX_ANG_VEL, TRACK_WIDTH), drive.getAccelerationConstraint(MAX_ACCEL))
                        .build();
                drive.followTrajectorySequence(trajSeqParking);
            }
            else if (parkingZone==ATRobotEnumeration.PARK2){
                trajSeqParking=drive.trajectorySequenceBuilder(drive.getPoseEstimate())
                        .lineToConstantHeading(new Vector2d(36, 11))//,drive.getVelocityConstraint(45, MAX_ANG_VEL, TRACK_WIDTH), drive.getAccelerationConstraint(MAX_ACCEL))
                        .build();
                drive.followTrajectorySequence(trajSeqParking);
            }
            else if (parkingZone==ATRobotEnumeration.PARK3){
                trajSeqParking=drive.trajectorySequenceBuilder(drive.getPoseEstimate())
                        .lineToConstantHeading(new Vector2d(10, 11))//,drive.getVelocityConstraint(45, MAX_ANG_VEL, TRACK_WIDTH), drive.getAccelerationConstraint(MAX_ACCEL))
                        .build();
                drive.followTrajectorySequence(trajSeqParking);
            }
            else if (parkingZone==ATRobotEnumeration.SUBSTATION){
                trajSeqParking=drive.trajectorySequenceBuilder(drive.getPoseEstimate())
                        .lineToConstantHeading(new Vector2d(10, 11))//,drive.getVelocityConstraint(45, MAX_ANG_VEL, TRACK_WIDTH), drive.getAccelerationConstraint(MAX_ACCEL))
                        .lineToSplineHeading(new Pose2d(10, 60, Math.toRadians(90)))
                        .build();
                drive.followTrajectorySequence(trajSeqParking);
            }
        setRobotStateInStorage();

        poseEstimate = drive.getPoseEstimate();
        telemetry.addData("x", poseEstimate.getX());
        telemetry.addData("y", poseEstimate.getY());
        telemetry.addData("heading", Math.toDegrees(poseEstimate.getHeading()));
        telemetry.addData("robot mode", tophatController.getTophatAction());
        telemetry.addData("Parking Zone", parkingZone);
        telemetry.addData("Get Runtime", this.getRuntime());
        telemetry.update();
    }
    private void setRobotStateInStorage(){
        ATGlobalStorage.autonModeName=ATRobotEnumeration.BLUE_LEFT_HIGH_DROP;
        ATGlobalStorage.currentPose=drive.getPoseEstimate();
        ATGlobalStorage.parkingPos=parkingZone;
        ATGlobalStorage.allianceName=ATRobotEnumeration.BLUE_ALLIANCE;
    }
}
