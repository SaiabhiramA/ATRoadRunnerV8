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
import org.firstinspires.ftc.teamcode.drive.TopHatAutoControllerStates;
import org.firstinspires.ftc.teamcode.trajectorysequence.TrajectorySequence;

@Config
//@Autonomous(group = "drive")
@Autonomous(name = "AprilTag")
@Disabled
public class BlueAllianceLeftHighDropAprilTag extends LinearOpMode {
    MecanumDriveAT drive;
    TopHatAutoControllerStates tophatController;
    ATAprilTag ATObjectDetection;
    ATRobotEnumeration parkingZone=ATRobotEnumeration.SUBSTATION;
    Pose2d poseEstimate;
    boolean isAutonConePickupReady=false;
    @Override
    public void runOpMode() throws InterruptedException {
        tophatController=new TopHatAutoControllerStates();
        ATObjectDetection = new ATAprilTag();
        ATObjectDetection.initalizeTensorFlow(hardwareMap, telemetry, ATRobotEnumeration.AUTO_BLUE_LEFT_HIGH_SETUP);
        //parkingZone = ATObjectDetection.detectObjectLabel();
        //tophatController.fullyInitializeRobot(telemetry, gamepad1, gamepad2, ATRobotEnumeration.RESET, hardwareMap);
        drive = new MecanumDriveAT(hardwareMap);
        Pose2d startPose = new Pose2d(40, 60, Math.toRadians(270));
        drive.setPoseEstimate(startPose);
        //parkingZone=ATRobotEnumeration.SUBSTATION;
        setRobotStateInStorage();
        TrajectorySequence trajSeqConePickup = drive.trajectorySequenceBuilder(startPose)
                .addTemporalMarker(.001, ()->{
                    tophatController.setTophatAction(ATRobotEnumeration.SET_BLUE_LEFT_PRELOADED_CONE);
                    tophatController.blueAllianceLeftAutonHigh();})
                .waitSeconds(2.5)
                .splineToConstantHeading(new Vector2d(36,58), Math.toRadians(-90))
                .splineToConstantHeading(new Vector2d(36,30), Math.toRadians(-90),drive.getVelocityConstraint(20, MAX_ANG_VEL, TRACK_WIDTH), drive.getAccelerationConstraint(MAX_ACCEL))
                .splineToConstantHeading(new Vector2d(32.5,(20-ATConstants.BLUE_LEFT_HIGH_DROP_CONE_DROP_OFFSET_Y)), Math.toRadians(-90),drive.getVelocityConstraint(20, MAX_ANG_VEL, TRACK_WIDTH), drive.getAccelerationConstraint(MAX_ACCEL))
                .addTemporalMarker(6, ()->{
                    tophatController.setTophatAction(ATRobotEnumeration.DROP_BLUE_LEFT_PRELOADED_CONE);
                    tophatController.blueAllianceLeftAutonHigh();})
                .waitSeconds(3)
                .splineToConstantHeading(new Vector2d(33,5), Math.toRadians(-90),drive.getVelocityConstraint(20, MAX_ANG_VEL, TRACK_WIDTH), drive.getAccelerationConstraint(MAX_ACCEL))
                .splineToConstantHeading(new Vector2d(45,(11+ATConstants.BLUE_LEFT_HIGH_DROP_CONE_PICKUP_OFFSET_Y)), Math.toRadians(0))//,drive.getVelocityConstraint(20, MAX_ANG_VEL, TRACK_WIDTH), drive.getAccelerationConstraint(MAX_ACCEL))
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
        //drive.followTrajectorySequence(trajSeqConePickup);

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
                telemetry.addData("are FiveCones Done", tophatController.areFiveConesDone());
                telemetry.update();
            }
        }
        else {
            tophatController.moveTopHatPosition(-1, false, 4250*tophatController.armMultiplier, -1800*tophatController.elbowMultiplier, 945);
            while ((!isStopRequested()) && !tophatController.isTopHatMoveCompleted(4250*tophatController.armMultiplier,-1800*tophatController.elbowMultiplier,945)) {
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
        /*TrajectorySequence trajSeqParking;
            if (parkingZone==ATRobotEnumeration.PARK1){
                trajSeqParking=drive.trajectorySequenceBuilder(drive.getPoseEstimate())
                        .lineToConstantHeading(new Vector2d(60, 11),drive.getVelocityConstraint(45, MAX_ANG_VEL, TRACK_WIDTH), drive.getAccelerationConstraint(MAX_ACCEL))
                        .build();
                drive.followTrajectorySequence(trajSeqParking);
            }
            else if (parkingZone==ATRobotEnumeration.PARK2){
                trajSeqParking=drive.trajectorySequenceBuilder(drive.getPoseEstimate())
                        .lineToConstantHeading(new Vector2d(36, 11),drive.getVelocityConstraint(45, MAX_ANG_VEL, TRACK_WIDTH), drive.getAccelerationConstraint(MAX_ACCEL))
                        .build();
                drive.followTrajectorySequence(trajSeqParking);
            }
            else if (parkingZone==ATRobotEnumeration.PARK3){
                trajSeqParking=drive.trajectorySequenceBuilder(drive.getPoseEstimate())
                        .lineToConstantHeading(new Vector2d(10, 11),drive.getVelocityConstraint(45, MAX_ANG_VEL, TRACK_WIDTH), drive.getAccelerationConstraint(MAX_ACCEL))
                        .build();
                drive.followTrajectorySequence(trajSeqParking);
            }
            else if (parkingZone==ATRobotEnumeration.SUBSTATION){
                trajSeqParking=drive.trajectorySequenceBuilder(drive.getPoseEstimate())
                        .lineToConstantHeading(new Vector2d(10, 11),drive.getVelocityConstraint(45, MAX_ANG_VEL, TRACK_WIDTH), drive.getAccelerationConstraint(MAX_ACCEL))
                        .lineToSplineHeading(new Pose2d(10, 60, Math.toRadians(90)))
                        .build();
                drive.followTrajectorySequence(trajSeqParking);
            }*/
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
