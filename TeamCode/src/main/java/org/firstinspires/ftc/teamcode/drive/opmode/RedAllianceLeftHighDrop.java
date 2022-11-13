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

import org.firstinspires.ftc.teamcode.drive.ATGlobalStorage;
import org.firstinspires.ftc.teamcode.drive.ATRobotEnumeration;
import org.firstinspires.ftc.teamcode.drive.ATTensorFlowDefaultDetection;
import org.firstinspires.ftc.teamcode.drive.MecanumDriveAT;
import org.firstinspires.ftc.teamcode.drive.TopHatAutoController;
import org.firstinspires.ftc.teamcode.trajectorysequence.TrajectorySequence;

@Config
//@Autonomous(group = "drive")
@Autonomous(name = "RedAllianceLeftHighDropAT")
//@Disabled
public class RedAllianceLeftHighDrop extends LinearOpMode {
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
        ATObjectDetection.initalizeTensorFlow(hardwareMap, telemetry, ATRobotEnumeration.AUTO_RED_LEFT_HIGH_SETUP);
        parkingZone = ATObjectDetection.detectObjectLabel();
        tophatController.fullyInitializeRobot(telemetry, gamepad1, gamepad2, ATRobotEnumeration.RESET, hardwareMap);
        drive = new MecanumDriveAT(hardwareMap);
        Pose2d startPose = new Pose2d(-40, -60, Math.toRadians(90));
        drive.setPoseEstimate(startPose);
        parkingZone=ATRobotEnumeration.SUBSTATION;
        setRobotStateInStorage();
        TrajectorySequence trajSeqConePickup = drive.trajectorySequenceBuilder(startPose)
                .addTemporalMarker(.001, ()->{
                    tophatController.setRobotMode(ATRobotEnumeration.SET_RED_LEFT_PRELOADED_CONE);
                    tophatController.redAllianceLeftAutonHigh();})
                .waitSeconds(2.5)
                //Following is one Combination of dropping preloaded cone
                /*.splineToConstantHeading(new Vector2d(-36,-50), Math.toRadians(90))
                .splineToConstantHeading(new Vector2d(-36,-3), Math.toRadians(90),drive.getVelocityConstraint(35, MAX_ANG_VEL, TRACK_WIDTH), drive.getAccelerationConstraint(MAX_ACCEL))
                .splineToConstantHeading(new Vector2d(-33.5,-3), Math.toRadians(90))
                .waitSeconds(.5)
                .splineToConstantHeading(new Vector2d(-33,-3), Math.toRadians(90),drive.getVelocityConstraint(15, MAX_ANG_VEL, TRACK_WIDTH), drive.getAccelerationConstraint(MAX_ACCEL))
                .waitSeconds(.5)
                .addDisplacementMarker(()->{
                    tophatController.setRobotMode(ATRobotEnumeration.DROP_RED_LEFT_PRELOADED_CONE);
                    tophatController.redAllianceLeftAutonHigh();})
                .waitSeconds(1.5)
                .splineToConstantHeading(new Vector2d(-38,-10), Math.toRadians(180),drive.getVelocityConstraint(15, MAX_ANG_VEL, TRACK_WIDTH), drive.getAccelerationConstraint(MAX_ACCEL))
                .waitSeconds(.25)
                .splineToConstantHeading(new Vector2d(-45,-10), Math.toRadians(180),drive.getVelocityConstraint(15, MAX_ANG_VEL, TRACK_WIDTH), drive.getAccelerationConstraint(MAX_ACCEL))
                .waitSeconds(12)*/

                .splineToConstantHeading(new Vector2d(-36,-50), Math.toRadians(90))
                .splineToConstantHeading(new Vector2d(-33,-20), Math.toRadians(90),drive.getVelocityConstraint(20, MAX_ANG_VEL, TRACK_WIDTH), drive.getAccelerationConstraint(MAX_ACCEL))
                .waitSeconds(1)
                .addTemporalMarker(4.5, ()->{
                    tophatController.setRobotMode(ATRobotEnumeration.DROP_RED_LEFT_PRELOADED_CONE);
                    tophatController.redAllianceLeftAutonHigh();})
                .splineToConstantHeading(new Vector2d(-33,-5), Math.toRadians(180),drive.getVelocityConstraint(20, MAX_ANG_VEL, TRACK_WIDTH), drive.getAccelerationConstraint(MAX_ACCEL))
                .splineToConstantHeading(new Vector2d(-45,-10), Math.toRadians(180))
                .waitSeconds(.5)
                .build();

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

        if (isAutonConePickupReady) {
            tophatController.setRobotMode(ATRobotEnumeration.AUTO_RED_LEFT_HIGH_PICK_CONE);
            while ((!isStopRequested()) && !tophatController.areFiveConesDone()) {
                tophatController.redAllianceLeftAutonHigh();
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
        else {
            tophatController.moveTopHatPosition(-1, false, 4250, -1800, 945);
            while ((!isStopRequested()) && !tophatController.isTopHatMoveCompleted(4250,-1800,945)) {
                poseEstimate = drive.getPoseEstimate();
                telemetry.addData("x", poseEstimate.getX());
                telemetry.addData("y", poseEstimate.getY());
                telemetry.addData("heading", Math.toDegrees(poseEstimate.getHeading()));
                telemetry.addData("robot mode", tophatController.getRobotMode());
                telemetry.addData("In While Loop", "YES");
                telemetry.update();
            }
            tophatController.setRobotMode(ATRobotEnumeration.AUTO_RED_LEFT_HIGH_PARK);
        }
        TrajectorySequence trajSeqParking;
        if (parkingZone==ATRobotEnumeration.PARK1){
            trajSeqParking=drive.trajectorySequenceBuilder(drive.getPoseEstimate())
                    .lineToConstantHeading(new Vector2d(-60,-10),drive.getVelocityConstraint(45, MAX_ANG_VEL, TRACK_WIDTH), drive.getAccelerationConstraint(MAX_ACCEL))
                    .build();
            drive.followTrajectorySequence(trajSeqParking);
        }
        else if (parkingZone==ATRobotEnumeration.PARK2){
            trajSeqParking=drive.trajectorySequenceBuilder(drive.getPoseEstimate())
                    .lineToConstantHeading(new Vector2d(-36,-10),drive.getVelocityConstraint(45, MAX_ANG_VEL, TRACK_WIDTH), drive.getAccelerationConstraint(MAX_ACCEL))
                    .build();
            drive.followTrajectorySequence(trajSeqParking);
        }
        else if (parkingZone==ATRobotEnumeration.PARK3){
            trajSeqParking=drive.trajectorySequenceBuilder(drive.getPoseEstimate())
                    .lineToConstantHeading(new Vector2d(-12, -10),drive.getVelocityConstraint(45, MAX_ANG_VEL, TRACK_WIDTH), drive.getAccelerationConstraint(MAX_ACCEL))
                    .build();
            drive.followTrajectorySequence(trajSeqParking);
        }
        else if (parkingZone==ATRobotEnumeration.SUBSTATION){
            trajSeqParking=drive.trajectorySequenceBuilder(drive.getPoseEstimate())
                    .lineToConstantHeading(new Vector2d(-12, -10),drive.getVelocityConstraint(45, MAX_ANG_VEL, TRACK_WIDTH), drive.getAccelerationConstraint(MAX_ACCEL))
                    .lineToSplineHeading(new Pose2d(-12, -60, Math.toRadians(0)),drive.getVelocityConstraint(45, MAX_ANG_VEL, TRACK_WIDTH), drive.getAccelerationConstraint(MAX_ACCEL))
                    .build();
            drive.followTrajectorySequence(trajSeqParking);
        }
        setRobotStateInStorage();

        poseEstimate = drive.getPoseEstimate();
        telemetry.addData("x", poseEstimate.getX());
        telemetry.addData("y", poseEstimate.getY());
        telemetry.addData("heading", Math.toDegrees(poseEstimate.getHeading()));
        telemetry.addData("robot mode", tophatController.getRobotMode());
        telemetry.addData("Parking Zone", parkingZone);
        telemetry.addData("Get Runtime", this.getRuntime());
        telemetry.update();
    }
    private void setRobotStateInStorage(){
        ATGlobalStorage.autonModeName=ATRobotEnumeration.RED_LEFT_HIGH_DROP;
        ATGlobalStorage.currentPose=drive.getPoseEstimate();
        ATGlobalStorage.parkingPos=parkingZone;
        ATGlobalStorage.allianceName=ATRobotEnumeration.RED_ALLIANCE;
    }
}
